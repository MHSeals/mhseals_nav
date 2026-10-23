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
