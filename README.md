# State Space Search Algorithms

A performant and flexible implementation of the [A* search algorithm](https://en.wikipedia.org/wiki/A*_search_algorithm) for Go.

## Usage Example

### 1. Import the package
```go
import "github.com/sparemind/go-state-space-search"
```

### 2. Implement the State interface
A State encapsulates the state of some system. For example, for pathfinding the state may be the position of some agent in the
world, while for solving a [15 puzzle](https://en.wikipedia.org/wiki/15_puzzle) the state would be the layout of tiles.

States have two methods that must be implemented:

- `NextStates` returns the States directly reachable from this State, along with the Transition to move to that state and the cost of doing so.
- `EstimateCost` returns an estimate the cost to reach another state from this state. For the search to be optimal, this 
  should be a [consistent heuristic](https://en.wikipedia.org/wiki/Consistent_heuristic) (i.e. it never overestimates
  the combined estimated cost from any neighbor to the given state plus the cost of reaching that neighbor).

This example implements it for `Position`, which represents a location
in a discrete 2D world, in order to allow pathfinding through it. Note that States must be [comparable](https://golang.org/ref/spec#Comparison_operators).
```go
type Position struct {
    x int
    y int
}

func (p Position) NextStates() []search.StateTransition {
    neighbors := make([]search.StateTransition, 0)
    for _, direction := range Directions {
    	neighbor, moveCost, canMove := p.Move(direction)
    	if !canMove {
            continue
        }
        neighbors = append(neighbors, search.StateTransition{
            State:      neighbor,
            Transition: direction,
            Cost:       moveCost,
        })	
    }
    return neighbors
}

func (p Position) EstimateCost(state *State) float64 {
    other := (*state).(Position)	
    return p.ManhattanDistance(other)
}
```

### 3. Search between two states
A search will return the lowest cost path from one state to another.
The solution path contains all steps from the starting state up to but not including the goal.
Each step consists of a state, the transition to the next state in the path, and the cost of making the transition.
```go
start := NewPosition(0, 0)
goal := NewPosition(8, 2)
solution, cost, found := search.Search(start, goal)

printWorld(world)
printWorldWithSolution(world, solution)
printDirections(solution)
```

```
@..####..
....##..$
.##......
.....##..
########.

>>V####..
..>V##..$
.##>>>>>^
.....##..
########.

RRDRDRRRRRU
```
