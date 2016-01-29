# novalidate
'''
    An implementation of java.util.TimerTask which is needed for
    the PID Controller
    
    TODO: Java switched to the notifier, so should we?
'''

import threading
from ..timer import Timer

import logging

class TimerTask(threading.Thread):
    
    def __init__(self, name, period, task_fn):
        super().__init__(name=name, daemon=True)
        self.logger = logging.getLogger('wpilib.%s' % name)
        self.period = period
        self.task_fn = task_fn
        
        self.last_warning = -10
        
        self.stopped = False
    
    def cancel(self):
        self.stopped = True
        self.join()
    
    def run(self):
        
        self.logger.info("Timer task started")
        
        try:
            period = self.period
            wait_til = Timer.getFPGATimestamp() + period
            
            while not self.stopped:
                
                now = Timer.getFPGATimestamp()
                delay = wait_til - now
                
                if delay > 0:
                    Timer.delay(wait_til - now)
                else:
                    # only emit warning once per second
                    if now - self.last_warning > 1:
                        self.last_warning = now
                        self.logger.warn("Delay time was negative (%02f: too much going on?)", delay)
                
                if self.stopped:
                    break
                
                self.task_fn()
                
                wait_til += period
        finally:
            self.logger.info("Timer task exited")
