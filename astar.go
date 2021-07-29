// Package astar implements the A* search algorithm.
package astar

import (
	"container/heap"
)

// Search returns the lowest cost path from a start state to a goal state
// along with the path's total cost. It additionally returns a boolean flag
// of whether a path could be found.
func Search(start State, goal State) ([]StateTransition, float64, bool) {
	itemCache := make(map[State]*node)
	closedSet := make(map[State]bool)
	openSet := make(PriorityQueue, 0)
	heap.Push(&openSet, &node{
		state:  start,
		gscore: 0,
		fscore: start.EstimateCost(&goal),
	})
	for len(openSet) > 0 {
		current := heap.Pop(&openSet).(*node)
		if closedSet[current.state] {
			continue
		}
		if current.state == goal {
			solution := make([]StateTransition, 0)
			solutionCost := current.gscore
			var transition interface{}
			if current.parent != nil {
				transition = current.transition
				current = current.parent
			}
			for current.parent != nil {
				solution = append(solution, StateTransition{
					State:      current.state,
					Transition: transition,
				})
				transition = current.transition
				current = current.parent
			}
			solution = append(solution, StateTransition{
				State:      start,
				Transition: transition,
			})
			// Reverse solution since it was constructed in reverse
			for i, j := 0, len(solution)-1; i < j; i, j = i+1, j-1 {
				solution[i], solution[j] = solution[j], solution[i]
			}
			return solution, solutionCost, true
		}
		closedSet[current.state] = true
		for _, next := range current.state.NextStates() {
			if closedSet[next.State] {
				continue
			}
			tentativeGScore := current.gscore + next.Cost
			if nextItem, exists := itemCache[next.State]; !exists || tentativeGScore < nextItem.gscore {
				if !exists {
					nextItem = &node{
						state: next.State,
					}
					itemCache[next.State] = nextItem
				}
				nextItem.fscore = tentativeGScore + next.State.EstimateCost(&goal)
				nextItem.gscore = tentativeGScore
				nextItem.cost = nextItem.fscore
				nextItem.parent = current
				nextItem.transition = next.Transition
				if exists {
					heap.Fix(&openSet, nextItem.index)
				} else {
					heap.Push(&openSet, nextItem)
				}
			}
		}
	}
	return nil, 0, false
}
