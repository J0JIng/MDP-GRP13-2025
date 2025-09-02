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
    """Formatter that injects emoji + (optional) color with no fixed-width padding."""

    EMOJIS = {
        logging.DEBUG: "🐞",
        logging.INFO: "ℹ️",
        logging.WARNING: "⚠️",
        logging.ERROR: "❌",
        logging.CRITICAL: "🔥",
    }

    def __init__(self, fmt: str, *, colorize: bool = True):
        super().__init__(fmt)
        self.colorize = colorize

    def format(self, record: logging.LogRecord) -> str:
        base_level = record.levelname  # e.g. 'INFO'
        emoji = self.EMOJIS.get(record.levelno, "")
        raw = f"{emoji} {base_level}".strip()
        if self.colorize:
            color = ANSI_COLORS.get(record.levelno, "")
            record.leveltext = f"{color}{raw}{ANSI_RESET}"
        else:
            record.leveltext = raw
        return super().format(record)


def prepare_logger(
    name: str = __name__,
    level: int = logging.DEBUG,
    clickable_paths: bool = True,
    aligned: bool = True,
    colorize: bool = True,
    level_width: int = 12,  # kept for backward compatibility, now unused
) -> logging.Logger:
    # Use current working directory for logs
    log_dir = Path.cwd() / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)

    log_filename = log_dir / datetime.now().strftime("log_%Y-%m-%d_%H-%M-%S.log")

    logger = logging.getLogger(name)
    logger.setLevel(level)

    if not logger.handlers:
        # Build format strings; console uses leveltext (colored), file uses plain text
        if clickable_paths:
            console_pattern = "%(leveltext)s :: %(name)s.%(funcName)s :: %(message)s :: %(pathname)s:%(lineno)d:"
            file_pattern = console_pattern
        else:
            console_pattern = "%(asctime)s :: %(leveltext)s :: %(name)s.%(funcName)s :: %(message)s"
            file_pattern = console_pattern

        # Console (color if requested & TTY)
        console_handler = logging.StreamHandler()
        use_color = colorize and getattr(console_handler.stream, "isatty", lambda: False)()
        console_handler.setFormatter(EmojiFormatter(console_pattern, colorize=use_color))
        console_handler.setLevel(level)
        logger.addHandler(console_handler)

        # File (no color)
        file_handler = logging.FileHandler(log_filename, mode="w", encoding="utf-8")
        file_handler.setFormatter(EmojiFormatter(file_pattern, colorize=False))
        file_handler.setLevel(level)
        logger.addHandler(file_handler)

    return logger
