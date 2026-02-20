from typing import Any, Tuple


#!moving the elemnt idx to the root to maintain heap

def sift_up(heap, idx):
    while idx > 0:
        parent = (idx - 1) >> 1
        if heap[idx] < heap[parent]:
            #swap
            heap[idx], heap[parent] = heap[parent], heap[idx]
            idx = parent
        else:
            break



def sift_down(heap, idx, size):
    while True:
        smallest = idx
        left = 2 * idx + 1
        right = 2 * idx + 2
        if left < size and heap[left] < heap[smallest]:
            smallest = left
        if right < size and heap[right] < heap[smallest]:
            smallest = right
        if smallest == idx:
            break
        heap[idx], heap[smallest] = heap[smallest], heap[idx]
        idx = smallest




class CustomPQ_maxG:

    #?we need push, we need pop, we need peek, decreaseKey, contains, peek, isEmpty, etcetera

    #?push(state, f, g)
    #?break ties by insertion order, if f-values are the same

    #?what do we actually need for the customPQ

    def __init__(self):
        self._heap: list = [] #? this is a list of states, which is (f, -g, counter, state)
        self._counter: int = 0 #?this is the counter we need to use to tiebreak
        self._index: dict = {} #? this is a map mapping state to heap position
        self._best: dict = {} #? this maps state to (f, -g)
    
    def is_empty(self) -> bool: #? empty check
        return len(self._heap) == 0

    def size(self) -> int: #? size function
        return len(self._heap)

    def contains(self, state: Tuple[int, int]) -> bool: #? self explanatory
        return state in self._index

        
    def push(self, state: Any, f: float, g:float) -> None:
        evilG = -g
        key = (f, evilG)
        if state in self._best and self._best[state] <= key:
            return
        #otherwise, we want to update the key and append
        self._best[state] = key
        entry = (f, evilG, self._counter, state)
        self._counter += 1
        self._heap.append(entry)
        idx = len(self._heap) - 1
        self._index[state] = idx
        sift_up(self._heap, idx)
        self._rebuild_index_around(idx)
    
    #!we only want to push if and only if the state has better priority than the best


    def pop(self) -> Tuple[Any, float, float]:
        if not self._heap:
            raise IndexError("erm nothing in the heap idiot")
        heap = self._heap

        heap[0], heap[-1] = heap[-1], heap[0]
        f, negg_g, _, state = heap.pop()
        self._index.pop(state, None)

        

    def _rebuild_index_around(self, idx: int) -> None:
        """
        After a sift operation the positions of several nodes may have
        changed.  We do a full linear scan only when necessary; for typical
        heap sizes this is acceptable.  A production implementation would
        maintain the index map inside every swap.
        """
        for i, entry in enumerate(self._heap):
            self._index[entry[3]] = i