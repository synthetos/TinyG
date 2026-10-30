"""E052: the RX ISR answers ENQ (0x05) with printf from inside the MED-level interrupt.

xio_usb.c's RX ISR calls controller_request_enquiry(), which printf_P()s {"ack":true} while the main
loop may be halfway through printing another response, so the two can interleave. The race is timing
dependent, so this is a stress guard rather than a strict reproduction: ENQ bytes are interleaved
with status-report requests and every line received must be valid JSON, with one ack per ENQ.
"""
import json
import time

import pytest

pytestmark = pytest.mark.dialect("d449")


def test_enq_acks_never_interleave_with_responses(tg):
    tg.drain(quiet=0.2)
    mark = len(tg.transport.history)
    n = 40
    for _ in range(n):
        tg.transport.write(b'{"sr":null}\n\x05')
        time.sleep(0.03)
    tg.drain(quiet=0.8)
    lines = [e.raw for e in tg.transport.history[mark:] if e.raw.strip()]
    bad = []
    for line in lines:
        try:
            json.loads(line)
        except ValueError:
            bad.append(line)
    acks = sum(1 for line in lines if '"ack"' in line)
    assert not bad, f"{len(bad)} corrupted lines, e.g. {bad[:3]}"
    assert acks == n, f"{acks} acks for {n} ENQ"
