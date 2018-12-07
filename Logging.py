from threading import Thread, Event
from Queue import Queue
import sys

import csv

class Logger(Thread):
    def __init__(self,name,stopper,queue):
        Thread.__init__(self)
        self.stopper = stopper
        self.queue = queue
        self.csvFile = open(name,'wb')
        self.writer = csv.writer(self.csvFile)

    def run(self):
        while not self.stopper.is_set():
            if not self.queue.empty():
                v = self.queue.get()
                self.writer.writerow(v)
                self.queue.task_done()
    
        self.csvFile.close()