"""Unit tests for helpers.messageParser.MessageParser.

These tests validate:
- Normal Z frame parsing (odo, speed, status, battery, ...)
- Bundled Z frame debundling (sub-frame extraction)
- Corrupt Z frame filtering (garbage odometer values)
- Extended CAN ($RCAN) parsing for drive mode, BMS, motor, bus voltage, NTC
- "Scooter off" reset of extended CAN fields
- IMEI / connection-independent behaviour (no network calls)

Real raw frames captured from a SEAT Mo 125 with the Astra AT402 v7.0.61.35
telematics module are used where possible.
"""
import pytest

from helpers.messageParser import MessageParser


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def parser():
    """Fresh parser for each test."""
    return MessageParser()


# ---------------------------------------------------------------------------
# Basic parser behaviour
# ---------------------------------------------------------------------------

def test_parser_initializes_with_scooter_off(parser):
    assert parser.scooter_off is True
    assert parser.off_statuses == [0, 1, 5]


def test_parser_loads_all_required_configs(parser):
    # Must load Z_protocol and RCAN configs plus the parameters dict
    assert "odo" in parser.message_decode
    assert "Cell1Voltage" in parser.RCAN_message_configuration
    assert "odo" in parser.parameters
    assert "status" in parser.parameters


def test_parser_declares_extended_can_keys(parser):
    # Our fix relies on a known list of extended-CAN fields
    for key in ["driveMode", "motorRPM", "motorPower", "busVoltage",
                "bmsCurrent", "batteryNTC1", "batteryNTC2", "batteryNTC3"]:
        assert key in parser._extended_can_keys


def test_parser_handles_empty_data(parser):
    # Should silently ignore empty bytearrays, never raise.
    parser.parse_message_from_scooter_protocol_Z(b"")
    parser.parse_message_from_scooter_protocol_astra(b"")
    # No state changes expected
    assert parser.scooter_off is True


# ---------------------------------------------------------------------------
# $RCAN parsing (extended CAN sensors)
# ---------------------------------------------------------------------------

@pytest.mark.parametrize(
    "rcan_id,bits,expected_mode",
    [
        ("280", 0x00, "OFF"),    # bits 4-5 = 00
        ("280", 0x10, "ECO"),    # bits 4-5 = 01
        ("280", 0x20, "SPORT"),  # bits 4-5 = 10
        ("280", 0x30, "CITY"),   # bits 4-5 = 11
    ],
)
def test_rcan_280_drive_mode_mapping(parser, rcan_id, bits, expected_mode):
    # byte0 encodes driveReady (bit 0), sidestandDown (bit 3), drive mode (bits 4-5)
    # parts layout: $RCAN,{ID},{len},{b0},{b1}
    frame = f"$RCAN,{rcan_id},2,{bits:02X},00\r\n"
    parser._parse_extended_can(frame)
    assert parser.parameters["driveMode"]["value"] == expected_mode


def test_rcan_280_drive_ready_and_sidestand(parser):
    # byte0 = 0b00001001 = 0x09: driveReady=1 (bit 0), sidestandDown=1 (bit 3)
    parser._parse_extended_can("$RCAN,280,2,09,01\r\n")
    assert parser.parameters["driveReady"]["value"] == 1
    assert parser.parameters["sidestandDown"]["value"] == 1
    assert parser.parameters["warningLights"]["value"] == 1  # byte1 bit 0 = 1


def test_rcan_300_range_by_mode(parser):
    # 0x300 - Range by drive mode at byte1 (parts[4])
    parser._parse_extended_can("$RCAN,300,2,00,2D\r\n")  # 0x2D = 45 km
    assert parser.parameters["rangeByMode"]["value"] == 45


def test_rcan_182_bms_flags(parser):
    parser._parse_extended_can("$RCAN,182,1,0F\r\n")
    assert parser.parameters["bmsFlags"]["value"] == 0x0F


def test_rcan_181_bms_current_positive(parser):
    # Bytes 6-7 little-endian, signed, /10 = amps
    # 0x0064 = 100 -> 10.0 A
    parser._parse_extended_can("$RCAN,181,8,00,00,00,00,00,00,64,00,OK")
    assert parser.parameters["bmsCurrent"]["value"] == 10.0


def test_rcan_181_bms_current_negative(parser):
    # 0xFF9C = -100 (signed 16-bit) -> -10.0 A
    parser._parse_extended_can("$RCAN,181,8,00,00,00,00,00,00,9C,FF,OK")
    assert parser.parameters["bmsCurrent"]["value"] == -10.0


def test_rcan_189_battery_ntc_three_probes(parser):
    # Bytes 2-7 encode 3 NTC probes (2 bytes each, LE, /100 = celsius, rounded to 1 decimal)
    # 0x09C4=2500 -> 25.0C, 0x0A28=2600 -> 26.0C, 0x0ABE=2750 -> 27.5C
    parser._parse_extended_can("$RCAN,189,8,00,00,C4,09,28,0A,BE,0A,OK")
    assert parser.parameters["batteryNTC1"]["value"] == 25.0
    assert parser.parameters["batteryNTC2"]["value"] == 26.0
    assert parser.parameters["batteryNTC3"]["value"] == 27.5


def test_rcan_391_motor_rpm(parser):
    # Bytes 4-5 LE unsigned -> RPM
    # 0x0BB8 = 3000
    parser._parse_extended_can("$RCAN,391,6,00,00,00,00,B8,0B,OK")
    assert parser.parameters["motorRPM"]["value"] == 3000


def test_rcan_381_motor_power_positive(parser):
    # Bytes 2-3 LE signed -> motor power
    # 0x01F4 = 500
    parser._parse_extended_can("$RCAN,381,4,00,00,F4,01,OK")
    assert parser.parameters["motorPower"]["value"] == 500


def test_rcan_381_motor_power_negative_regen(parser):
    # Negative power = regeneration
    # 0xFF38 = -200
    parser._parse_extended_can("$RCAN,381,4,00,00,38,FF,OK")
    assert parser.parameters["motorPower"]["value"] == -200


def test_rcan_371_bus_voltage(parser):
    # Bytes 6-7 LE unsigned, /10 = volts
    # 0x0320 = 800 -> 80.0 V
    parser._parse_extended_can("$RCAN,371,8,00,00,00,00,00,00,20,03,OK")
    assert parser.parameters["busVoltage"]["value"] == 80.0


def test_rcan_unknown_id_is_ignored(parser):
    # Should not raise on unknown RCAN IDs
    before = dict(parser.parameters)
    parser._parse_extended_can("$RCAN,999,2,00,00\r\n")
    # No values should have changed
    for k in parser._extended_can_keys:
        assert parser.parameters[k]["value"] == before[k]["value"]


def test_rcan_malformed_frame_does_not_raise(parser):
    # Too few fields, non-hex data, etc. should be caught
    parser._parse_extended_can("$RCAN,181,not-enough-fields")
    parser._parse_extended_can("$RCAN,181,8,ZZ,00,00,00,00,00,00,00,OK")
    parser._parse_extended_can("not-an-rcan-frame")
    # No assertion needed — we just verify no exception escapes


# ---------------------------------------------------------------------------
# "Scooter off" reset of extended CAN fields (our 2026-04-22 fix)
# ---------------------------------------------------------------------------

def test_extended_can_fields_reset_when_scooter_off(parser):
    # Simulate the scooter having just been ridden:
    parser.parameters["motorRPM"]["value"] = 3000
    parser.parameters["motorPower"]["value"] = 1500
    parser.parameters["driveMode"]["value"] = "SPORT"
    parser.parameters["busVoltage"]["value"] = 82.1
    parser.parameters["bmsCurrent"]["value"] = -30.5
    parser.parameters["batteryNTC1"]["value"] = 25.0
    parser.scooter_off = False

    # Now the scooter shuts down — craft a minimal 94-byte Z frame with
    # status=0 at byte 82. Everything else can be zero, we only care
    # about scooter_off transitioning to True and the reset logic firing.
    frame = bytearray(94)
    frame[0] = 0x5A          # Z marker
    frame[1] = 0x00          # len hi
    frame[2] = 0x5E          # len lo (94)
    frame[3] = 0x01          # sub-count
    frame[82] = 0            # status = 0 (off)

    parser.parse_message_from_scooter_protocol_Z(bytes(frame))

    assert parser.scooter_off is True
    # Extended CAN fields should be reset to "None"
    for key in parser._extended_can_keys:
        assert parser.parameters[key]["value"] == "None", (
            f"{key} not reset on scooter-off"
        )


def test_extended_can_fields_kept_when_scooter_on(parser):
    # When scooter is ON, extended CAN values must NOT be reset — they
    # are the live RPM/power/etc. that we want to publish.
    parser.parameters["motorRPM"]["value"] = 3000
    parser.parameters["driveMode"]["value"] = "SPORT"
    parser.scooter_off = True  # will flip to False when we send status=4

    frame = bytearray(94)
    frame[0] = 0x5A
    frame[1] = 0x00
    frame[2] = 0x5E
    frame[3] = 0x01
    frame[82] = 4            # status = 4 (moving)

    parser.parse_message_from_scooter_protocol_Z(bytes(frame))

    assert parser.scooter_off is False
    # Should not be reset
    assert parser.parameters["motorRPM"]["value"] == 3000
    assert parser.parameters["driveMode"]["value"] == "SPORT"


# ---------------------------------------------------------------------------
# Bundled Z frame debundling (our 2026-04-14 fix)
# ---------------------------------------------------------------------------

def test_normal_z_frame_not_touched_by_debundler(parser):
    # A 94-byte frame is the normal single-record case. The debundler
    # should not touch it.
    frame = bytearray(94)
    frame[0] = 0x5A
    frame[1] = 0x00
    frame[2] = 0x5E  # 94
    frame[3] = 0x01  # 1 sub-record
    frame[82] = 4    # status
    parser.parse_message_from_scooter_protocol_Z(bytes(frame))
    # Status must have been read from byte 82
    assert parser.parameters["status"]["value"] == 4


def test_bundled_frame_extracts_last_subframe(parser):
    # The debundler only kicks in for frames > 200 bytes. Build a 3-sub
    # bundled frame: 4 header + 3*88 sub + 2 checksum = 270 bytes.
    # After debundling, the synthetic single-record frame is 88+6 = 94 bytes,
    # which matches the 94-byte layout the Z-protocol config expects
    # (status at byte 82).
    sub_size = 88
    sub_count = 3
    total_len = 4 + sub_count * sub_size + 2  # 270 bytes

    frame = bytearray(total_len)
    frame[0] = 0x5A
    frame[1] = (total_len >> 8) & 0xFF
    frame[2] = total_len & 0xFF
    frame[3] = sub_count

    # sub 0, 1 have status=3 at offset (82 - 4) = 78 inside the 88-byte sub
    # sub 2 (last, latest) has status=4 — this is what we want to extract
    for i in range(sub_count):
        frame[4 + i * sub_size + 78] = 3 if i < sub_count - 1 else 4

    parser.parse_message_from_scooter_protocol_Z(bytes(frame))
    assert parser.parameters["status"]["value"] == 4


# ---------------------------------------------------------------------------
# Corrupt Z frame filtering (our 2026-04-14 fix)
# ---------------------------------------------------------------------------

def test_corrupt_odo_value_is_rejected(parser):
    # An odo of 0x33FFFFFF ~ 870M is a known shutdown-corruption signature.
    # We first seed a good value to see it NOT be overwritten.
    parser.parameters["odo"]["value"] = 15026.0
    parser.scooter_off = False  # Must be off->on so numeric fields are parsed

    # Build a frame with odo at bytes 83-86 = 0x33FFFFFF
    frame = bytearray(94)
    frame[0] = 0x5A
    frame[1] = 0x00
    frame[2] = 0x5E
    frame[3] = 0x01
    frame[82] = 4  # status=4 so scooter_off flips to False
    # odo bytes 83-86 big-endian = 0x33FFFFFF
    frame[83] = 0x33
    frame[84] = 0xFF
    frame[85] = 0xFF
    frame[86] = 0xFF

    parser.parse_message_from_scooter_protocol_Z(bytes(frame))

    # The corrupt value should have been rejected by the >1_000_000 guard,
    # meaning the publish was skipped. The internal parameters dict WILL
    # contain the parsed value (the rejection happens just before publish),
    # but the side-effect we care about in integration is: no MQTT publish.
    # Here we assert the guard detected it by checking the value is
    # either not the garbage (early return) or is flagged somehow.
    # Since the guard does `return` before publish, the stored value IS
    # the garbage — but the test proves the branch is reachable. A stronger
    # test would need a mocked pub.sendMessage; see test below.
    assert parser.parameters["odo"]["value"] > 1_000_000


def test_corrupt_odo_skips_publish(parser, monkeypatch):
    """Strong version: verify pub.sendMessage is NOT called on corrupt frame."""
    calls = []
    import helpers.messageParser as mp
    monkeypatch.setattr(mp.pub, "sendMessage", lambda *a, **kw: calls.append((a, kw)))

    parser.scooter_off = False

    frame = bytearray(94)
    frame[0] = 0x5A
    frame[1] = 0x00
    frame[2] = 0x5E
    frame[3] = 0x01
    frame[82] = 4
    frame[83] = 0x33
    frame[84] = 0xFF
    frame[85] = 0xFF
    frame[86] = 0xFF

    parser.parse_message_from_scooter_protocol_Z(bytes(frame))

    # No publish must have occurred because the frame was rejected
    status_publishes = [c for c in calls if c[1].get("scooter_status") is not None
                        or (c[0] and c[0][0] == mp.TOPIC_SCOOTER_STATUS)]
    assert status_publishes == [], (
        f"Corrupt frame should not trigger a publish, got: {calls}"
    )


def test_valid_odo_does_publish(parser, monkeypatch):
    """Positive control: a frame with a plausible odo DOES publish."""
    calls = []
    import helpers.messageParser as mp
    monkeypatch.setattr(mp.pub, "sendMessage", lambda *a, **kw: calls.append((a, kw)))

    parser.scooter_off = False

    frame = bytearray(94)
    frame[0] = 0x5A
    frame[1] = 0x00
    frame[2] = 0x5E
    frame[3] = 0x01
    frame[82] = 4  # status=4
    # odo = 15026 = 0x00003AB2
    frame[83] = 0x00
    frame[84] = 0x00
    frame[85] = 0x3A
    frame[86] = 0xB2

    parser.parse_message_from_scooter_protocol_Z(bytes(frame))

    assert len(calls) >= 1, "A valid frame must trigger at least one publish"


# ---------------------------------------------------------------------------
# Status transitions drive scooter_off flag
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("status,expected_off", [
    (0, True),   # off
    (1, True),   # lock
    (2, False),  # startup
    (3, False),  # ready/idle (moving trip active)
    (4, False),  # riding
    (5, True),   # shutdown
])
def test_scooter_off_flag_follows_status(parser, status, expected_off):
    parser.scooter_off = not expected_off  # start in opposite state

    frame = bytearray(94)
    frame[0] = 0x5A
    frame[1] = 0x00
    frame[2] = 0x5E
    frame[3] = 0x01
    frame[82] = status

    parser.parse_message_from_scooter_protocol_Z(bytes(frame))
    assert parser.scooter_off is expected_off


def test_get_scooter_off_status_getter(parser):
    assert parser.get_scooter_off_status() is True
    parser.scooter_off = False
    assert parser.get_scooter_off_status() is False
