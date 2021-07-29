package astar

// node represents a state being explored by A*.
type node struct {
	state      State
	transition interface{}
	gscore     float64
	cost       float64 // The cost of the node in the queue (lowest at front).
	parent     *node   // Previous state in the lowest cost path to this state.
	index      int     // The index of the node in the heap (used for heap fixing).
}

// A PriorityQueue implements heap.Interface and holds state nodes.
type PriorityQueue []*node

func (p PriorityQueue) Len() int {
	return len(p)
}

func (p PriorityQueue) Less(i int, j int) bool {
	return p[i].cost < p[j].cost
}

func (p PriorityQueue) Swap(i, j int) {
	p[i], p[j] = p[j], p[i]
	p[i].index = i
	p[j].index = j
}

func (p *PriorityQueue) Push(x interface{}) {
	n := len(*p)
	item := x.(*node)
	item.index = n
	*p = append(*p, item)
}

func (p *PriorityQueue) Pop() interface{} {
	old := *p
	n := len(old)
	item := old[n-1]
	*p = old[0 : n-1]
	return item
}
