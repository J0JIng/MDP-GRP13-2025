import logging
from datetime import datetime


class EmojiFormatter(logging.Formatter):
    """
    Custom formatter that adds emojis for different log levels.
    """
    EMOJIS = {
        logging.DEBUG: "🐞",    # Debug
        logging.INFO: "ℹ️",     # Info
        logging.WARNING: "⚠️",  # Warning
        logging.ERROR: "❌",    # Error
        logging.CRITICAL: "🔥",  # Critical
    }

    def format(self, record: logging.LogRecord) -> str:
        emoji = self.EMOJIS.get(record.levelno, "")
        record.levelname = f"{emoji} {record.levelname}"
        return super().format(record)


def prepare_logger(
    name: str = __name__,
    level: int = logging.DEBUG,
) -> logging.Logger:
    """
    Creates a logger that prints to console and saves to a timestamped .log file.
    Each run of the program gets its own log file, with emojis in the console/file logs.
    """
    # Generate unique log filename with timestamp
    log_filename = datetime.now().strftime("log_%Y-%m-%d_%H-%M-%S.log")

    logger = logging.getLogger(name)
    logger.setLevel(level)

    if not logger.hasHandlers():
        log_format = EmojiFormatter(
            "%(asctime)s :: %(levelname)s :: %(name)s.%(funcName)s:%(lineno)d :: %(message)s"
        )

        # Console handler
        console_handler = logging.StreamHandler()
        console_handler.setLevel(level)
        console_handler.setFormatter(log_format)
        logger.addHandler(console_handler)

        # File handler (timestamped .log file)
        file_handler = logging.FileHandler(log_filename, mode="w", encoding="utf-8")
        file_handler.setLevel(level)
        file_handler.setFormatter(log_format)
        logger.addHandler(file_handler)

    return logger
