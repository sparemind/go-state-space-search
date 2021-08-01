// Package astar implements the A* search algorithm.
package astar

import (
	"container/heap"
)

// Search returns the lowest cost path from a start state to a goal state
// along with the path's total cost. It additionally returns a boolean flag
// of whether a path could be found. The returned path consists of an ordered
// slice of StateTransitions for all steps in the solution path, except for
// the goal, starting from the start state. Each step consists of the state,
// the transition to the next state in the path, and the cost of making the
// transition.
func Search(start State, goal State) ([]StateTransition, float64, bool) {
	nodes := make(map[State]*node)
	openSet := make(PriorityQueue, 0)
	heap.Push(&openSet, &node{
		state:             start,
		costFromStart:     0,
		estimatedPathCost: start.EstimateCost(&goal),
		open:              true,
	})
	for len(openSet) > 0 {
		current := heap.Pop(&openSet).(*node)
		if current.state == goal {
			pathCost := current.costFromStart
			path := reconstructPath(current)
			return path, pathCost, true
		}
		current.open = false
		current.closed = true
		for _, next := range current.state.NextStates() {
			nextNode, exists := nodes[next.State]
			if !exists {
				nextNode = &node{
					state: next.State,
				}
				nodes[next.State] = nextNode
			}
			if nextNode.closed {
				continue
			}
			costFromStart := current.costFromStart + next.Cost
			if !exists || costFromStart < nextNode.costFromStart {
				nextNode.estimatedPathCost = costFromStart + next.State.EstimateCost(&goal)
				nextNode.costFromStart = costFromStart
				nextNode.parent = current
				if nextNode.open {
					heap.Fix(&openSet, nextNode.index)
				} else {
					heap.Push(&openSet, nextNode)
				}
			}
		}
	}
	return nil, 0, false
}

func reconstructPath(goal *node) []StateTransition {
	// Find all nodes in the solution path
	goalToStartPath := make([]*node, 0)
	current := goal
	for current != nil {
		goalToStartPath = append(goalToStartPath, current)
		current = current.parent
	}

	// Reconstruct the path from start to goal. Since NextStates()
	// should be a cheap operation we run it here to get the
	// transition and cost of each step (as opposed to saving them
	// in the main Search() as part of the nodes), which reduces
	// the memory usage of nodes.
	path := make([]StateTransition, 0, len(goalToStartPath))
	for i := len(goalToStartPath) - 1; i > 0; i-- {
		current := goalToStartPath[i]
		next := goalToStartPath[i-1]

		nextStates := current.state.NextStates()
		for _, nextState := range nextStates {
			if nextState.State == next.state {
				path = append(path, StateTransition{
					State:      current.state,
					Transition: nextState.Transition,
					Cost:       nextState.Cost,
				})
				break
			}
		}
	}

	return path
}
