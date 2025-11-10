from threading import Thread
from random import randint
from time import sleep


class testClass:
    def __init__(self):
        self.list1 = []
        self.dict1 = {}
        self.bool1 = False

    def __str__(self):
        return f"List1: {self.list1}\nDict1: {self.dict1}\nBool1: {self.bool1}"


class dummyController:
    def __init__(self, shared_obj):
        self.shared_obj = shared_obj

    def modify(self, list1, dict1, bool1):
        self.shared_obj.list1 = list1
        self.shared_obj.dict1 = dict1
        self.shared_obj.bool1 = bool1


def tester(shared_obj):
    controller = dummyController(shared_obj)

    controller.modify(
        [randint(0, 10), randint(0, 10), randint(0, 10)],
        {"test1": randint(0, 1), "test2": randint(0, 1), "test3": randint(0, 1)},
        True,
    )
    return


def looptester(shared_obj):
    for i in range(10):
        tester(shared_obj)
        sleep(0.1)


theClass = testClass()
print(theClass)

modifier_thread = Thread(target=looptester, args=(theClass,), daemon=True)
modifier_thread.start()

print(theClass)
modifier_thread.join()
print(theClass)
