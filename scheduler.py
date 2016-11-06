import Queue
import threading
import time

class Scheduler:
    def __init__(self):
        self._queue = Queue.Queue(maxsize=50)
        self._event_loop = threading.Thread(target=self._run)
        self._tasks = {}
        self._events = {}
        self._stop = threading.Event()
        self._eventId = 0
        self._started = False

    def _run(self):
        pending = False
        while True:
            event = self._queue.get()
            if event is None:
                pending = True    
            else:
                self._handle_event(event)
            self._queue.task_done()
            if pending and self._queue.empty():
                break
    
    def _handle_event(self, id):
        handler, args = self._events[id]
        handler(*args)
    
    def add_event(self, handler=None, args=()):
        _id = None
        if handler != None:
            _id = self._eventId
            self._eventId += 1
            self._events[_id] = (handler, args)
        return _id

    def postEvent(self, id):
        self._queue.put(id)
            
    def add_task(self, handler, interval = 0, args=()):
        _id = None
        if handler != None:
            _id = self._eventId
            self._eventId += 1
            thread = threading.Thread(target=self._run_task, args=(_id,))
            self._tasks[_id] = (thread, interval)
            self._events[_id] = (handler, args)
        return _id

    def _run_task(self, id):
        thread, interval = self._tasks[id]
        if interval == 0:
            self.postEvent(id)
            return
        while not self._stop.is_set():
            self.postEvent(id)
            time.sleep(interval)
     
    def start(self):
        if self._started:
            return
        self._started = True
        self._event_loop.start()
        for key, task in self._tasks.iteritems():
            thread, interval = task
            thread.start()

    def stop(self):
        self._stop.set()
        self._queue.put(None)
        self._queue.join()
        self._event_loop.join()
        for key, task in self._tasks.iteritems():
            thread, interval = task
            thread.join()
   


# Self-test code
def _test():
    data = [1,2,3]
    mini = [4,5,6]
    mustafa = [7,8,9]
    scheduler = Scheduler()
    kTask1 = scheduler.add_task(_task, 0.2, (data, mini))
    kEvent_Render = scheduler.add_event(_render, (data, mini))
    kTask2 = scheduler.add_task(_task, 0, (data, mustafa))
    scheduler.start()
    try:
        while True:
            scheduler.postEvent(kEvent_Render)
            time.sleep(3)
    except KeyboardInterrupt:
        print ("attempting to close threads.")
        scheduler.stop()
        print ("terminated.")

def _render(d,m):
    print ("redering", d, m)

def _task(d,m):
    print ("a task", d, m)

if __name__ == '__main__':
    _test()


