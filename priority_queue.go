package astar

// node represents a state being explored by A*.
type node struct {
	state             State
	open              bool
	closed            bool
	costFromStart     float64 // Best known cost from starting state to this state
	estimatedPathCost float64 // Estimate of total path cost from start to goal that goes through this state
	parent            *node   // Previous state in the lowest cost path to this state
	index             int     // The index of the node in the heap (used for heap fixing)
}

// A PriorityQueue implements heap.Interface and holds state nodes, ordered in increasing estimated path cost.
type PriorityQueue []*node

func (p PriorityQueue) Len() int {
	return len(p)
}

func (p PriorityQueue) Less(i int, j int) bool {
	return p[i].estimatedPathCost < p[j].estimatedPathCost
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
