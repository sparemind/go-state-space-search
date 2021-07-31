package astar_test

import (
	"fmt"
	"github.com/sparemind/go-astar"
	"math/rand"
	"strings"
	"testing"
)

// assertSolution causes a test failure if the lowest cost solution for a
// pathfinding scenario doesn't match the expected cost.
//
// This function can also accept optional arguments:
// - A position, which will override any specified in the given world string.
// - A string of transition directions representing the expected path, which will
//   be checked against the found path.
func assertSolution(
	t *testing.T,
	expectedCost float64,
	worldString string,
	args ...interface{},
) []astar.StateTransition {
	world, start, goal := newWorld(worldString)

	var expectedPathTransitions *string = nil
	for _, arg := range args {
		switch arg := arg.(type) {
		case position:
			goal.x, goal.y = arg.x, arg.y
		case string:
			expectedPathTransitions = &arg
		}
	}

	solution, cost, found := astar.Search(start, goal)
	if !found && expectedCost >= 0 {
		t.Fatalf("expected solution with cost %f, no solution found", expectedCost)
	}
	if found && cost != expectedCost {
		fmt.Println(world.SolutionString(solution))
		t.Fatalf("expected solution with cost %f, found solution with cost %f", expectedCost, cost)
	}
	if pathTransitions := pathTransitions(solution); expectedPathTransitions != nil &&
		pathTransitions != *expectedPathTransitions {
		t.Fatalf("expected solution with directions %s, was %s",
			*expectedPathTransitions,
			pathTransitions)
	}
	return solution
}

func pathTransitions(path []astar.StateTransition) string {
	builder := strings.Builder{}
	for _, step := range path {
		builder.WriteString(step.Transition.(string))
	}
	return builder.String()
}

func TestSearch_Search_NoSolution(t *testing.T) {
	assertSolution(t, -1, `
		..#..
		@.#.*
		..#..
	`)
	assertSolution(t, -1, `
		...#
		@.#*
		...#
	`)
	assertSolution(t, -1, `
		#...
		@#.*
		#...
	`)
}

func TestSearch_EmptySolution(t *testing.T) {
	path := assertSolution(t, 0, `
		111
		1@1
		111
	`, position{x: 1, y: 1})
	if len(path) != 0 {
		t.Fatalf("expected solution path to be empty, was %d states long", len(path))
	}
}

func TestSearch_StraightPath(t *testing.T) {
	assertSolution(t, 6, `
		.@.....*.
	`, ">>>>>>")
	assertSolution(t, 4, `
		.....
		@...*
		.....
	`, ">>>>")
	assertSolution(t, 4, `
		.....
		*...@
		.....
	`, "<<<<")
}

func TestSearch_AvoidHighCost(t *testing.T) {
	assertSolution(t, 6, `
		.....
		@.9.*
		.....
	`)
	assertSolution(t, 12, `
		@.594.*
		..643..
		..28...
		.......
	`)
}

func TestSearch_TakeShortcut(t *testing.T) {
	assertSolution(t, 7, `
		@.4.*
		..9..
		.....
	`, ">>>>")
	assertSolution(t, 10, `
		@..23..*
		...789..
		...456..
		........
	`, ">>>>>>>")
}

func TestSearch_AvoidBacktracking(t *testing.T) {
	assertSolution(t, 16, `
		.........
		.######..
		......#..
		......#..
		@.....#.*
		......#..
		......#..
		.######..
		.........
	`)
}

func TestSearch_LongPath(t *testing.T) {
	assertSolution(t, 18, `
		@..2..2....2...
		.2.2..22.2...2.
		.2.........2...
		..2.222222..222
		2.........2...*
	`, ">>VV>>>>>>>>V>V>>>")
}

func BenchmarkSearch(b *testing.B) {
	rand.Seed(0)
	worldSize := 256

	world := make(world, worldSize)
	for y := 0; y < worldSize; y++ {
		world[y] = make([]int, worldSize)
		for x := 0; x < worldSize; x++ {
			world[y][x] = rand.Intn(9) + 1
		}
	}
	start := newPosition(0, 0, &world)
	goal := newPosition(worldSize-1, worldSize-1, &world)

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		astar.Search(start, goal)
	}
}
