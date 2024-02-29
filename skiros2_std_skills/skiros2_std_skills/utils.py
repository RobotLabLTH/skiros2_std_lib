import queue
from threading import Lock
from typing import Optional

class AtomicVar:
    def __init__(self, initialValue: Optional[any]=None, sharedLock: Optional[Lock] = None) -> None:
        self.lock = sharedLock
        self.var = initialValue
        if self.lock is None:
            self.lock = Lock()
    
    def get_and_reset(self):
        with self.lock:
            var = self.var
            if var is not None:
                self.var = None
            return var

    def get(self):
        with self.lock:
            return self.var

    def set(self, var):
        with self.lock:
            self.var = var