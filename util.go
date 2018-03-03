package main

import "container/heap"

func toStringPointer(str string) *string {
	return &str
}

type PriorityQueueNode struct {
	value    *Point
	priority float64
	index  int
	parent *PriorityQueueNode
}

// A PriorityQueue implements heap.Interface and holds Items.
type PriorityQueue []*PriorityQueueNode

func (pq PriorityQueue) Len() int { return len(pq) }

func (pq PriorityQueue) Less(i, j int) bool {
	// We want Pop to give us the highest, not lowest, priority so we use greater than here.
	return pq[i].priority < pq[j].priority
}

func (pq PriorityQueue) Swap(i, j int) {
	pq[i], pq[j] = pq[j], pq[i]
	pq[i].index = i
	pq[j].index = j
}

func (pq *PriorityQueue) Push(x interface{}) {
	n := len(*pq)
	item := x.(*PriorityQueueNode)
	item.index = n
	*pq = append(*pq, item)
}

func (pq *PriorityQueue) Pop() interface{} {
	old := *pq
	n := len(old)
	item := old[n-1]
	item.index = -1 // for safety
	*pq = old[0 : n-1]
	return item
}

// update modifies the priority and value of an Item in the queue.
func (pq *PriorityQueue) update(item *PriorityQueueNode, value *Point, priority float64) {
	item.value = value
	item.priority = priority
	heap.Fix(pq, item.index)
}
