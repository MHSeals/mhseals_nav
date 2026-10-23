"""Message rates must be requested again whenever the FCU forgets them."""
from mhseals_nav.mavros_setup import parse_rates, request_rates


def test_rates_parse_to_id_and_hz():
    assert parse_rates(['33:20.0', '193:2']) == [(33, 20.), (193, 2.)]


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


def test_rc_channels_is_requested_at_25_hz_by_default():
    from pathlib import Path
    launch = (Path(__file__).parents[1] / 'launch' / 'odom.launch.py').read_text()
    assert "(65, LaunchConfiguration('mavros_rc_rate')" in launch
    assert "('mavros_rc_rate', '25.0'" in launch
