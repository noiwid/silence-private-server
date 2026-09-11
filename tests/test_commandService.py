"""Unit tests for services.CommandService.

Regression tests for the `imei` -> `self.IMEI` fix: every logging /
result-reporting path in CommandService referenced a bare `imei` name
that was never defined, so each of these methods raised
`NameError: name 'imei' is not defined` as soon as it ran — tearing down
the scooter listener thread right after a command was sent.

These tests exercise all five affected paths (command_received,
cleanup_queue, get_next_command, command_executed, command_failed) and
assert that no exception escapes and that results are published with the
correct IMEI.
"""
import pytest

from helpers.command import Command
from services.CommandService import CommandService
import services.CommandService as cs_mod


TEST_IMEI = "860873043967941"

CMD_CONFIG = {"command": "$STMS\r\n", "maxretry": 3, "timeout": 60}


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def published(monkeypatch):
    """Capture pub.sendMessage calls as (args, kwargs) tuples."""
    calls = []
    monkeypatch.setattr(cs_mod.pub, "sendMessage",
                        lambda *a, **kw: calls.append((a, kw)))
    return calls


@pytest.fixture
def service():
    svc = CommandService(configuration={}, IMEI=TEST_IMEI)
    # start() loads this from disk and spawns a thread; unit tests
    # inject the definition directly instead.
    svc.commands_definition = {"SYNC": CMD_CONFIG}
    return svc


def make_queued_command(service, code="SYNC"):
    """Put a command in the queue and take it, mirroring the real flow
    (get_next_command -> execute -> command_executed/command_failed)."""
    cmd = Command(code, CMD_CONFIG)
    service.command_queue.put(cmd)
    service.command_queue.get()
    return cmd


# ---------------------------------------------------------------------------
# command_received
# ---------------------------------------------------------------------------

def test_command_received_enqueues_without_nameerror(service, published):
    # Used to raise NameError on the "Command inserted in queue" log line.
    service.command_received("SYNC", "")
    assert service.command_queue.qsize() == 1


def test_command_received_rejects_unknown_command(service, published):
    service.command_received("NOT_A_COMMAND", "")
    assert service.command_queue.qsize() == 0
    assert any("Invalid command" in kw.get("result", "")
               for _, kw in published)


# ---------------------------------------------------------------------------
# command_executed / command_failed
# ---------------------------------------------------------------------------

def test_command_executed_publishes_result_with_imei(service, published):
    # Used to raise NameError while formatting the result message,
    # killing the scooter listener thread after every executed command.
    cmd = make_queued_command(service)
    service.command_executed(cmd, b"$STMS,0,82,25,24,556")

    assert len(published) == 1
    _, kw = published[0]
    assert kw["command"] == "SYNC"
    assert TEST_IMEI in kw["result"]


def test_command_failed_requeues_for_retry(service, published):
    cmd = make_queued_command(service)
    service.command_failed(cmd)  # retry_attempts 0 -> 1, below maxretry

    assert service.command_queue.qsize() == 1
    assert published == []  # no result published while retries remain


def test_command_failed_reports_retry_limit_with_imei(service, published):
    cmd = make_queued_command(service)
    cmd.retry_attempts = cmd.maxretry  # next retry() returns False

    service.command_failed(cmd)

    assert service.command_queue.qsize() == 0
    assert len(published) == 1
    _, kw = published[0]
    assert "retry limit" in kw["result"].lower()
    assert TEST_IMEI in kw["result"]


# ---------------------------------------------------------------------------
# Timeout paths (cleanup_queue / get_next_command)
# ---------------------------------------------------------------------------

def expired_command():
    cmd = Command("SYNC", CMD_CONFIG)
    cmd.TSInserted -= CMD_CONFIG["timeout"] + 10
    return cmd


def test_cleanup_queue_drops_expired_command(service, published):
    service.command_queue.put(expired_command())

    service.cleanup_queue()

    assert service.command_queue.qsize() == 0
    assert any("timeout" in kw.get("result", "").lower() and TEST_IMEI in kw["result"]
               for _, kw in published)


def test_get_next_command_reports_expired_command(service, published):
    service.command_queue.put(expired_command())

    cmd = service.get_next_command()

    # Current behaviour: the expired command is still returned (the
    # caller re-checks the timeout), but the result must be published
    # without raising.
    assert cmd is not None
    assert any("timeout" in kw.get("result", "").lower()
               for _, kw in published)


def test_get_next_command_empty_queue_returns_none(service):
    assert service.get_next_command() is None
