"""Message rates must be requested again whenever the FCU forgets them."""
import struct

from mhseals_nav.mavros_setup import (
    decode_boot_ms, decode_ekf_status, ekf_status_values, parse_rates, rebooted,
    request_rates, with_removed)
import pytest


def test_rates_parse_to_id_and_hz():
    assert parse_rates(['33:20.0', '193:2']) == [(33, 20.), (193, 2.)]


def test_negative_rate_disables_and_non_finite_is_refused():
    assert parse_rates(['33:-1']) == [(33, -1.)]
    for bad in ('33:nan', '33:inf', '-1:5'):
        with pytest.raises(ValueError):
            parse_rates([bad])


def test_removed_ids_are_reset_to_the_fcu_default():
    assert with_removed([(33, 20.), (1, 2.)], [(33, 10.)]) == [(33, 10.), (1, 0.)]
    assert with_removed([], [(33, 10.)]) == [(33, 10.)]


def test_a_raising_call_is_logged_and_does_not_stop_later_rates():
    lines = []

    def call(message_id, hz):
        if message_id == 65:
            raise RuntimeError('service call failed')
        return True

    request_rates([(65, 25.), (33, 20.)], call, lines.append, attempts=2, sleep=lambda _: None)
    assert 'message 33 at 20 Hz: ok' in lines
    assert 'message 65 at 25 Hz: failed' in lines


def test_a_failed_request_is_retried_then_reported():
    calls, lines, waits = [], [], []
    replies = {33: [False, True], 24: [False, False, False]}

    def call(message_id, hz):
        calls.append(message_id)
        return replies[message_id].pop(0)

    request_rates([(33, 20.), (24, 5.)], call, lines.append, sleep=waits.append)
    assert calls == [33, 33, 24, 24, 24]
    assert lines == ['message 33 at 20 Hz: ok', 'message 24 at 5 Hz: failed']
    assert waits == [8., 8., 8.]


def test_rates_are_requested_again_only_on_reconnect():
    from mhseals_nav.mavros_setup import reconnected
    states = [False, True, True, False, True]
    edges = [reconnected(a, b) for a, b in zip([False] + states, states)]
    assert edges == [False, True, False, False, True]


def test_rates_are_not_also_requested_unconditionally_at_start():
    import inspect
    from mhseals_nav import mavros_setup
    assert 'wanted.set()  # once at start' not in inspect.getsource(mavros_setup.main)


def test_rc_channels_is_requested_at_25_hz_by_default():
    from pathlib import Path
    launch = (Path(__file__).parents[1] / 'launch' / 'odom.launch.py').read_text()
    assert "(65, LaunchConfiguration('mavros_rc_rate')" in launch
    assert "('mavros_rc_rate', '25.0'" in launch


# EKF_STATUS_REPORT as sent on the boat: flags 167 = ATT|VEL_H|VEL_V|POS_V_ABS|CONST_POS_MODE.
EKF_PAYLOAD = struct.pack('<5fHf', 0.1, 0.2, 0.3, 0.4, 0.5, 167, 0.)


def words(raw):
    raw = raw.ljust(-(-len(raw) // 8) * 8, b'\0')
    return list(struct.unpack(f'<{len(raw) // 8}Q', raw))


def test_ekf_status_payload_decodes_flags_and_variances():
    status = decode_ekf_status(193, len(EKF_PAYLOAD), words(EKF_PAYLOAD))
    assert status['flags'] == 167
    assert status['names'] == ['ATTITUDE', 'VELOCITY_HORIZ', 'VELOCITY_VERT',
                               'POS_VERT_ABS', 'CONST_POS_MODE']
    assert status['compass_variance'] == pytest.approx(0.4, rel=1e-6)
    assert status['pos_horiz_variance'] == pytest.approx(0.2, rel=1e-6)


def test_a_truncated_mavlink2_payload_is_zero_padded():
    short = EKF_PAYLOAD[:22]  # the extension field was all zeros and was cut
    assert decode_ekf_status(193, 22, words(short))['flags'] == 167


def test_other_messages_publish_nothing():
    assert decode_ekf_status(24, len(EKF_PAYLOAD), words(EKF_PAYLOAD)) is None


def test_const_pos_mode_warns_and_a_healthy_ekf_is_ok():
    stuck = decode_ekf_status(193, 26, words(EKF_PAYLOAD))
    level, message, values = ekf_status_values(stuck)
    assert (level, values['const_pos_mode'], values['flags']) == (1, 'true', '167')
    assert message.endswith('CONST_POS_MODE')
    good = decode_ekf_status(193, 26, words(struct.pack('<5fHf', 0, 0, 0, 0, 0, 1 | 16 | 32, 0)))
    assert ekf_status_values(good)[0] == 0
    assert ekf_status_values(good)[2]['const_pos_mode'] == 'false'


def test_high_flag_bits_use_the_ardupilot_names():
    raw = struct.pack('<5fHf', 0, 0, 0, 0, 0, 1024 | 32768, 0)
    status = decode_ekf_status(193, len(raw), words(raw))
    assert status['names'] == ['UNINITIALIZED', 'GPS_GLITCHING']


def test_a_time_boot_drop_means_the_fcu_rebooted_without_disconnecting():
    payload = struct.pack('<QI', 0, 90_000)
    assert decode_boot_ms(2, len(payload), words(payload)) == 90_000
    assert decode_boot_ms(193, len(payload), words(payload)) is None
    assert rebooted(90_000, 1_500)
    assert not rebooted(90_000, 91_000)
    assert not rebooted(None, 1_500)


def test_rc_at_the_fcu_default_rate_is_flagged_low_and_25_hz_is_not():
    from collections import deque
    from mhseals_nav.mavros_setup import rc_rate_low
    slow = deque((t * 0.5, 16) for t in range(11))
    fast = deque((t * 0.04, 16) for t in range(126))
    assert rc_rate_low(slow, 5.)
    assert not rc_rate_low(fast, 5.)


def test_rc_frames_without_channels_do_not_count_and_silence_is_low():
    from collections import deque
    from mhseals_nav.mavros_setup import rc_rate_low
    empty = deque((t * 0.04, 0) for t in range(126))
    assert rc_rate_low(empty, 5.)
    assert rc_rate_low(deque(), 100.)
