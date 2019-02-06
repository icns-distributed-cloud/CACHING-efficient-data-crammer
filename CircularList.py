
class CircularList:
    def __init__(self, list_size):
        self.__list = [None] * list_size
        self.__index = 0

    def put(self, data):
        self.__list[self.__index] = data
        self.__index = (self.__index + 1) % self.list_size

    @property
    def list(self):
        return self.__list
