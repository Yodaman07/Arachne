import threading


class A:
    def __init__(self):
        self.pause_event = threading.Event()

    def controller_mixin(self):  # points is a String to be parsed
        print("Init Thread")
        # Pt Key:
        # Formatted like {"point": ___}, Options are: "NA", "(_,_)" <- actual point
        while True:
            self.pause_event.wait()
            print("wwww")

    def abc(self):
        autonomousThread = threading.Thread(target=self.controller_mixin)
        autonomousThread.start()
        self.pause_event.clear()  # by default start thread and then pause it
        input("aaa")
        self.pause_event.set()
        input("bbb")
        self.pause_event.clear()
        print("awesome")

a = A()
a.abc()
