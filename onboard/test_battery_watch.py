"""Tests for battery_watch.py: python -m pytest onboard/test_battery_watch.py"""
from battery_watch import HALT_AFTER_S, STALE_S, Reading, Watch, guard_reading, parse, should_halt, step

STATUS = "I (123) main: {top} | {bottom} | wifi -60 dBm | ros up, 0 failed publishes | wheels 0.00 0.00 m/s"


def status(top, bottom):
    return STATUS.format(top=top, bottom=bottom)


def test_parse_status_lines():
    assert parse(status("11.87V -0.42A", "ROS OK 5.0W")) == Reading("OK", 11.87, -0.42)
    assert parse(status("10.45V 0.30A", "BATT LOW")) == Reading("LOW", 10.45, 0.30)
    assert parse(status("10.01V 0.15A", "BATT CRITICAL")) == Reading("CRITICAL", 10.01, 0.15)
    assert parse(status("11.90V", "AGENT...")) == Reading("OK", 11.90, None)   # INA228 missing: ADC volts
    assert parse(status("2.52V -0.06A", "WIFI...")) == Reading("NO PACK", 2.52, -0.06)


def test_parse_state_change():
    line = "W (99) main: battery LOW -> CRITICAL (10.05 V at 0.30 A, 10.15 V resting)"
    assert parse(line) == Reading("CRITICAL", 10.05, 0.30)
    assert parse("W (99) main: battery CRITICAL -> OK (12.20 V at 0.20 A, 12.10 V resting)").state == "OK"


def test_parse_ignores_other_lines():
    assert parse("I (5) main: lidarbot_base phase 2") is None
    assert parse("") is None
    assert parse("\x00\xff garbage 12V |") is None


def run(readings):
    w, t = Watch(), 0.0
    for state, dt in readings:
        t += dt
        w = step(w, Reading(state, 10.0, 0.2), t)
    return w, t


def test_halts_after_sustained_critical():
    w, t = run([("OK", 0)] + [("CRITICAL", 10)] * 6)
    assert not should_halt(w, t)                       # 50 s since the first critical line
    w, t = run([("OK", 0)] + [("CRITICAL", 10)] * 7)
    assert should_halt(w, t)


def test_recovery_resets_the_timer():
    w, t = run([("CRITICAL", 10)] * 5 + [("LOW", 10)] + [("CRITICAL", 10)] * 3)
    assert not should_halt(w, t + 10)


def test_silence_does_not_halt():
    w, t = run([("CRITICAL", 10)] * 3)
    assert not should_halt(w, t + HALT_AFTER_S)        # serial went quiet: no fresh reading
    w = step(w, Reading("CRITICAL", 10.0, 0.2), t + STALE_S + 60)
    assert not should_halt(w, t + STALE_S + 60)        # a gap restarts the count


def test_no_pack_never_halts():
    w, t = run([("NO PACK", 10)] * 20)
    assert not should_halt(w, t)


def test_guard_states_from_ros():
    assert guard_reading(3, 10.0, 0.2) == Reading("CRITICAL", 10.0, 0.2)
    assert guard_reading(1, 12.1, None).state == "OK"
    assert guard_reading(0, 0.0, None).state == "NO PACK"
    assert guard_reading(9, 12.0, None) is None
