import heapq

class PriorityQueue:

    def __init__(self):
        """
        Class constructor.
        """
        self.elements = []

    def empty(self):
        """
        Returns True if the queue is empty, False otherwise.
        """
        return len(self.elements) == 0

    def put(self, element, priority):
        """
        Puts an element in the queue.
        :param element  [any type]     The element.
        :param priority [int or float] The priority.
        """
        heapq.heappush(self.elements, (priority, element))

    def remove(self, element):
        # why doesn't python have a remove function which doesn't
        # except on inputs that aren't in the list...  
        try:
            self.elements.remove(element)
        except:
            pass

    def get(self):
        """
        Returns and removes the element with the top priority.
        """
        return heapq.heappop(self.elements)[1]
    
    def get_queue(self):
        """
        Returns the content of the queue as a list.
	"""
        return self.elements