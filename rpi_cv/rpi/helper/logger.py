from typing import Optional
import logging
from datetime import datetime
from pathlib import Path


ANSI_RESET = "\033[0m"
ANSI_COLORS = {
    logging.DEBUG: "\033[36m",      # Cyan
    logging.INFO: "\033[32m",       # Green
    logging.WARNING: "\033[33m",    # Yellow
    logging.ERROR: "\033[31m",      # Red
    logging.CRITICAL: "\033[97;41m"  # White on Red background
}


class EmojiFormatter(logging.Formatter):
    """Formatter that applies optional ANSI color to the log level (no emojis)."""

    def __init__(self, fmt: str, *, colorize: bool = True):
        super().__init__(fmt)
        self.colorize = colorize

    def format(self, record: logging.LogRecord) -> str:
        raw = record.levelname  # e.g. 'INFO'
        if self.colorize:
            color = ANSI_COLORS.get(record.levelno, "")
            record.leveltext = f"{color}{raw}{ANSI_RESET}"
        else:
            record.leveltext = raw
        return super().format(record)


# Shared handler state (one file & one console for the whole process)

_SHARED_FILE_HANDLER: Optional[logging.Handler] = None
_SHARED_CONSOLE_HANDLER: Optional[logging.Handler] = None
_SESSION_TS = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
_SHARED_LOG_PATH: Optional[Path] = None


def prepare_logger(
    name: str = __name__,
    level: int = logging.DEBUG,
    clickable_paths: bool = True,
    colorize: bool = True,
) -> logging.Logger:
    """Return a logger that writes into a single shared session log file.

    All calls in the process share the same file & console handlers to avoid
    per-class log files. Handler levels are set to DEBUG; individual logger
    levels still control emission.
    """
    global _SHARED_FILE_HANDLER, _SHARED_CONSOLE_HANDLER, _SHARED_LOG_PATH

    logger = logging.getLogger(name)
    logger.setLevel(level)
    logger.propagate = False  # we manage handlers explicitly

    # Build format strings once (same for all handlers)
    if clickable_paths:
        pattern = "%(leveltext)s :: %(name)s.%(funcName)s :: %(message)s :: %(pathname)s:%(lineno)d:"
    else:
        pattern = "%(asctime)s :: %(leveltext)s :: %(name)s.%(funcName)s :: %(message)s"

    # Ensure shared handlers exist
    if _SHARED_FILE_HANDLER is None or _SHARED_CONSOLE_HANDLER is None:
        log_dir = Path.cwd() / "logs"
        log_dir.mkdir(parents=True, exist_ok=True)
        _SHARED_LOG_PATH = log_dir / f"log_{_SESSION_TS}.log"

        # Console
        console_handler = logging.StreamHandler()
        use_color = colorize and getattr(console_handler.stream, "isatty", lambda: False)()
        console_handler.setFormatter(EmojiFormatter(pattern, colorize=use_color))
        console_handler.setLevel(logging.DEBUG)
        _SHARED_CONSOLE_HANDLER = console_handler

        # File (no color)
        file_handler = logging.FileHandler(_SHARED_LOG_PATH, mode="w", encoding="utf-8")
        file_handler.setFormatter(EmojiFormatter(pattern, colorize=False))
        file_handler.setLevel(logging.DEBUG)
        _SHARED_FILE_HANDLER = file_handler

    # Attach shared handlers if this logger doesn't have them yet
    existing = {id(h) for h in logger.handlers}
    for h in (_SHARED_CONSOLE_HANDLER, _SHARED_FILE_HANDLER):
        if id(h) not in existing:
            logger.addHandler(h)  # type: ignore[arg-type]

    return logger
