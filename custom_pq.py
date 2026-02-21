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
        f, neg_g, _, state = heap.pop()
        self._index.pop(state, None)
        self._best.pop(state, None)
        if heap:
            self._index[heap[0][3]] = 0
            sift_down(heap, 0, len(heap))
            self._rebuild_index_around(0)
        return state, f, -neg_g
    

    def decrease_key(self, state: Any, f: float, g: float) -> None:
        """Update priority of *state* to (f, g) if it improves."""
        # Lazy approach: just push; stale entries are ignored on pop.
        # For correctness with the index map we do a proper update when
        # the state is already present.
        neg_g = -g
        key = (f, neg_g)
        if state in self._best:
            if self._best[state] <= key:
                return  # not an improvement
            self._best[state] = key
            idx = self._index[state]
            # Update the entry in place (counter stays the same is fine –
            # the new key is strictly better so it will rise above the old one)
            old = self._heap[idx]
            self._heap[idx] = (f, neg_g, old[2], state)
            sift_up(self._heap, idx)
            self._rebuild_index_around(idx)
        else:
            self.push(state, f, g)


        

    def _rebuild_index_around(self, idx: int) -> None:
        """
        After a sift operation the positions of several nodes may have
        changed.  We do a full linear scan only when necessary; for typical
        heap sizes this is acceptable.  A production implementation would
        maintain the index map inside every swap.
        """
        for i, entry in enumerate(self._heap):
            self._index[entry[3]] = i
            


class CustomPQ_minG:
    """
    Min-heap that breaks ties among equal f-values in favor of SMALLER g-values.
    Stores (priority_tuple, cell) where priority_tuple = (f, g).
    """

    def __init__(self):
        self.heap: list[tuple[tuple[int, int], tuple[int, int]]] = []  # ((f, g), (r, c))
        self.pos: dict[tuple[int, int], int] = {}  # cell -> index in heap

    def is_empty(self) -> bool:
        return len(self.heap) == 0

    def size(self) -> int:
        return len(self.heap)

    def contains(self, cell: Tuple[int, int]) -> bool:
        return cell in self.pos

    def peek(self) -> Optional[Tuple[Tuple[int, int], Tuple[int, int]]]:
        if self.heap:
            return self.heap[0]
        return None

    def insert(self, cell: Tuple[int, int], f: int, g: int) -> None:
        """Insert cell with f-value and g-value. Tie-breaks by smaller g."""
        priority = (f, g)
        if cell in self.pos:
            self.update(cell, f, g)
            return
        self.heap.append((priority, cell))
        idx = len(self.heap) - 1
        self.pos[cell] = idx
        self._bubble_up(idx)

    def extract_min(self) -> Optional[Tuple[int, int]]:
        """Remove and return the cell with smallest (f, g). Returns None if empty."""
        if not self.heap:
            return None
        _, cell = self.heap[0]
        self._remove_at(0)
        return cell

    def remove(self, cell: Tuple[int, int]) -> None:
        """Remove a specific cell from the heap."""
        if cell not in self.pos:
            return
        idx = self.pos[cell]
        self._remove_at(idx)

    def update(self, cell: Tuple[int, int], f: int, g: int) -> None:
        """Update priority of an existing cell. Inserts if not present."""
        priority = (f, g)
        if cell not in self.pos:
            self.insert(cell, f, g)
            return
        idx = self.pos[cell]
        old_priority = self.heap[idx][0]
        self.heap[idx] = (priority, cell)
        if priority < old_priority:
            self._bubble_up(idx)
        else:
            self._bubble_down(idx)

    def clear(self) -> None:
        self.heap.clear()
        self.pos.clear()

    # -- internal helpers --

    def _swap(self, i: int, j: int) -> None:
        self.pos[self.heap[i][1]] = j
        self.pos[self.heap[j][1]] = i
        self.heap[i], self.heap[j] = self.heap[j], self.heap[i]

    def _bubble_up(self, idx: int) -> None:
        while idx > 0:
            parent = (idx - 1) // 2
            if self.heap[idx][0] < self.heap[parent][0]:
                self._swap(idx, parent)
                idx = parent
            else:
                break

    def _bubble_down(self, idx: int) -> None:
        n = len(self.heap)
        while True:
            smallest = idx
            left = 2 * idx + 1
            right = 2 * idx + 2
            if left < n and self.heap[left][0] < self.heap[smallest][0]:
                smallest = left
            if right < n and self.heap[right][0] < self.heap[smallest][0]:
                smallest = right
            if smallest != idx:
                self._swap(idx, smallest)
                idx = smallest
            else:
                break

    def _remove_at(self, idx: int) -> None:
        cell = self.heap[idx][1]
        del self.pos[cell]
        last = len(self.heap) - 1
        if idx == last:
            self.heap.pop()
            return
        last_item = self.heap.pop()
        self.heap[idx] = last_item
        self.pos[last_item[1]] = idx
        self._bubble_up(idx)
        self._bubble_down(idx)
