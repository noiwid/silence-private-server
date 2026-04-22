# Tests

Unit tests for the silence-private-server, focused on the message parser
(`helpers/messageParser.py`) — the component most prone to subtle breakage
when the scooter firmware or CAN layout changes.

## What is covered

- Normal Z-frame parsing (94-byte and bundled frames)
- Debundling of multi-record Z frames from the Astra AT402
- Rejection of corrupt Z frames (garbage odometer values seen at shutdown)
- Extended CAN parsing: `$RCAN,{280,300,182,181,189,371,381,391}`
- Drive mode decoding (`OFF` / `ECO` / `SPORT` / `CITY`)
- Signed-integer handling for motor power and BMS current (regen case)
- Scooter-off behaviour: reset of extended CAN fields, `scooter_off` flag
- Graceful handling of malformed input (no crashes on bad data)

34 tests run in well under a second.

## How to run

From the repo root:

```bash
pip install -r requirements.txt
pip install pytest
python -m pytest tests/ -v
```

Tests are also run automatically on every push and pull request via
GitHub Actions (see `.github/workflows/tests.yml`), on Python 3.9,
3.11 and 3.13.

## How to add a new test

1. Open `tests/test_messageParser.py`.
2. Group with related tests (extended CAN, debundling, etc.).
3. Prefer hand-crafted binary frames over captured ones — they make it
   obvious which bytes encode what. Helpers like `bytearray(94)` and
   direct index assignment (`frame[82] = 4`) keep the test readable.
4. If your test needs to assert on side effects of `pub.sendMessage`,
   use `monkeypatch.setattr(mp.pub, "sendMessage", ...)` — the
   `test_corrupt_odo_skips_publish` test shows the pattern.
