import logging
from helper.logger import prepare_logger
from pathlib import Path


def test_prepare_logger_creates_logfile(tmp_path, monkeypatch):
    # Run inside temp dir so logs directory is isolated
    monkeypatch.chdir(tmp_path)
    logger1 = prepare_logger("test.logger1")
    logger2 = prepare_logger("test.logger2")

    # Emit a log message
    logger1.info("hello world")

    # Find shared file handler & ensure log file written
    file_handlers = [h for h in logger1.handlers if isinstance(h, logging.FileHandler)]
    assert file_handlers, "Expected a file handler"
    log_path = Path(file_handlers[0].baseFilename)
    assert log_path.exists(), "Log file should exist"
    contents = log_path.read_text(encoding="utf-8")
    assert "hello world" in contents

    # Second logger should share same file handler
    file_handlers2 = [h for h in logger2.handlers if isinstance(h, logging.FileHandler)]
    assert file_handlers2
    assert file_handlers[0].baseFilename == file_handlers2[0].baseFilename
