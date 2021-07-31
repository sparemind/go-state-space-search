package astar_test

import (
	"github.com/sparemind/go-astar"
	"math"
	"strconv"
	"strings"
)

// A 2D world of navigable cells that have different movement costs.
type world [][]int

var directions = map[string][]int{
	"^": {0, -1},
	">": {1, 0},
	"V": {0, 1},
	"<": {-1, 0},
}

const (
	PathOne        = '.' // Same as '1', just used to make test strings more readable
	PathStart      = '@'
	PathGoal       = '*'
	PathImpassable = '#'
)

func (w *world) Height() int {
	return len(*w)
}

func (w *world) Width() int {
	return len((*w)[0])
}

func (w *world) String() string {
	builder := strings.Builder{}
	for _, row := range *w {
		for _, value := range row {
			builder.WriteString(strconv.Itoa(value))
		}
		builder.WriteRune('\n')
	}
	return builder.String()
}

func newWorld(worldString string) (*world, position, position) {
	rows := strings.Split(strings.TrimSpace(worldString), "\n")
	world := make(world, len(rows))
	var start, goal position
	start.world = &world
	goal.world = &world
	for y, row := range rows {
		row = strings.TrimSpace(row)
		world[y] = make([]int, len(row))
		for x, value := range row {
			if value == PathOne {
				world[y][x] = 1
			} else if value == PathStart {
				world[y][x] = 1
				start = newPosition(x, y, &world)
			} else if value == PathGoal {
				world[y][x] = 1
				goal = newPosition(x, y, &world)
			} else if value == PathImpassable {
				world[y][x] = -1
			} else {
				world[y][x] = int(value - '0')
			}
		}
	}
	return &world, start, goal
}

func (w *world) SolutionString(solution []astar.StateTransition) string {
	solutionSteps := make(map[position]string)
	for _, step := range solution {
		if step.Transition == nil {
			continue
		}
		solutionSteps[step.State.(position)] = step.Transition.(string)
	}

	builder := strings.Builder{}
	for y, row := range *w {
		for x, value := range row {
			if direction, ok := solutionSteps[position{x, y, w}]; ok {
				builder.WriteString(direction)
			} else if value < 0 {
				builder.WriteRune(PathImpassable)
			} else {
				builder.WriteString(strconv.Itoa(value))
			}
			builder.WriteRune(' ')
		}
		builder.WriteRune('\n')
	}
	return builder.String()
}

type position struct {
	x     int
	y     int
	world *world
}

func newPosition(x int, y int, world *world) position {
	return position{x: x, y: y, world: world}
}

func (p position) NextStates() []astar.StateTransition {
	nextStates := make([]astar.StateTransition, 0, 4)
	for direction, offset := range directions {
		nextX := p.x + offset[0]
		nextY := p.y + offset[1]
		if nextY < 0 || nextX < 0 || nextY >= p.world.Height() || nextX >= p.world.Width() {
			continue
		}
		cost := float64((*p.world)[nextY][nextX])
		if cost < 0 {
			continue
		}
		nextStates = append(nextStates, astar.StateTransition{
			State:      newPosition(nextX, nextY, p.world),
			Transition: direction,
			Cost:       cost,
		})
	}
	return nextStates
}

func (p position) EstimateCost(state *astar.State) float64 {
	other := (*state).(position)
	return math.Abs(float64(other.x-p.x)) + math.Abs(float64(other.y-p.y))
}
