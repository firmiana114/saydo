"""Bounded, immediately flushed diagnostics independent of upstream logging."""

import datetime
import json
import logging
from logging.handlers import RotatingFileHandler
import time


class EventLog:
    def __init__(self, path):
        self.logger = logging.Logger(str(path), level=logging.INFO)
        self.logger.propagate = False
        self.handler = RotatingFileHandler(
            path,
            maxBytes=5 * 1024 * 1024,
            backupCount=3,
            encoding='utf-8',
            delay=True,
        )
        self.handler.setFormatter(logging.Formatter('%(message)s'))
        self.logger.addHandler(self.handler)

    def write(self, event, **values):
        record = {
            'event': event,
            'at': time.time(),
            'utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
            **values,
        }
        self.logger.info(
            json.dumps(record, ensure_ascii=False, allow_nan=False)
        )
        return record

    def close(self):
        self.handler.close()

    def __del__(self):
        self.close()
