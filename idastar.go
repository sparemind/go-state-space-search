package search

import (
	"math"
)

func IterativeSearch(start State, goal State, args ...interface{}) ([]StateTransition, float64, bool) {
	if start == goal {
		return []StateTransition{}, 0, true
	}

	weight := 1.0
	estimateCost := State.EstimateCost
	for _, arg := range args {
		switch arg := arg.(type) {
		case float64:
			weight = arg
		case func(State, *State) float64:
			estimateCost = arg
		}
	}

	bound := estimateCost(start, &goal) * weight
	solution := []StateTransition{
		{
			State: start,
		},
	}
	solutionSet := make(map[State]bool)
	for {
		t, found := search(&solution, solutionSet, &goal, 0, bound, weight, estimateCost)
		if found {
			return solution[:len(solution)-1], bound, true
		}
		if t == math.MaxFloat64 {
			return nil, 0, false
		}
		bound = t
	}
}

func search(
	solution *[]StateTransition,
	solutionSet map[State]bool,
	goal *State,
	costFromStart float64,
	bound float64,
	weight float64,
	estimateCost func(State, *State) float64,
) (float64, bool) {
	lastIndex := len(*solution) - 1
	current := (*solution)[lastIndex]
	estimatedPathCost := costFromStart + estimateCost(current.State, goal)*weight
	if estimatedPathCost > bound {
		return estimatedPathCost, false
	}
	if current.State == *goal {
		return bound, true
	}
	min := math.MaxFloat64
	for _, next := range current.State.NextStates() {
		if !solutionSet[next.State] {
			*solution = append(*solution, next)
			solutionSet[next.State] = true

			t, found := search(solution, solutionSet, goal, costFromStart+next.Cost, bound, weight, estimateCost)
			if found {
				(*solution)[lastIndex].Transition = next.Transition
				(*solution)[lastIndex].Cost = next.Cost
				return min, true
			}
			if t < min {
				min = t
			}

			*solution = (*solution)[:len(*solution)-1]
			solutionSet[next.State] = false
		}
	}
	return min, false
}
