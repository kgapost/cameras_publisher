import logging
import logging.handlers
import os
import sys

# Log files are written next to the code (e.g. /workspace/logs inside the
# docker container), one file per module: logs/<module_name>.log.
# Mount a host volume on that folder to read them from outside the container
# (see README.md).
LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs')


def setup_logger(module_name):
    """Return a logger that writes to both the terminal (stdout) and
    logs/<module_name>.log. Safe to call more than once for the same module:
    handlers are only attached on the first call."""
    logger = logging.getLogger(module_name)
    if logger.handlers:
        return logger
    logger.setLevel(logging.DEBUG)   # handlers filter: terminal INFO, file DEBUG
    logger.propagate = False

    formatter = logging.Formatter(
        fmt='%(asctime)s [%(name)s] %(levelname)s: %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S')

    # Terminal output (stdout, like the print() calls this replaces - also what
    # 'docker logs' captures).
    stream_handler = logging.StreamHandler(sys.stdout)
    stream_handler.setLevel(logging.INFO)
    stream_handler.setFormatter(formatter)
    logger.addHandler(stream_handler)

    # File output: logs/<module_name>.log. If the folder cannot be created or
    # written (e.g. read-only bind mount), keep running with terminal-only
    # logging instead of crashing the module.
    try:
        os.makedirs(LOG_DIR, exist_ok=True)
        # Rotating (5 MB x 3 backups) so an unattended drone cannot fill its disk.
        file_handler = logging.handlers.RotatingFileHandler(
            os.path.join(LOG_DIR, module_name + '.log'), maxBytes=5_000_000, backupCount=3)
        file_handler.setLevel(logging.DEBUG)
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)
    except OSError as e:
        logger.error(f"Could not open log file for '{module_name}' in {LOG_DIR}: {e}. "
                     f"Continuing with terminal logging only.")
    return logger
