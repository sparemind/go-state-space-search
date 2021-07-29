package astar

// State represents a node in a weighted state graph. States are aware of their
// neighboring States that they can transition to. Implementing this interface
// enables shortest-path searches to be performed on the state graph.
type State interface {
	// NextStates returns the States directly reachable from this State.
	NextStates() []StateTransition
	// EstimateCost returns an estimate of how much it would cost to reach
	// another State from this State. This estimate doesn't overestimate the
	// actual cost it would take to reach it.
	EstimateCost(*State) float64
}

// StateTransition represents the transition to some
// State, with an associated cost for the transition.
type StateTransition struct {
	State      State       // The state being transitioned to
	Transition interface{} // The transition to reach this state from a previous state
	Cost       float64     // The cost of making this transition
}
