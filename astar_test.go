package search_test

import (
	"fmt"
	"github.com/sparemind/go-state-space-search"
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
) []search.StateTransition {
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

	solution, cost, found := search.Search(start, goal)
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

func pathTransitions(path []search.StateTransition) string {
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
	assertSolution(t, 15, `
		......#.
		......#.
		@.....#*
		......#.
		......#.
		.######.
		........
	`, "VVVV>>>>>>>^^^^")
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

func TestSearch_SuboptimalWeighting(t *testing.T) {
	world, start, goal := newWorld(`
		.....@999*
		.########.
		..........
	`)
	solution, cost, _ := search.Search(start, goal)
	if cost != 18 {
		fmt.Println(world.SolutionString(solution))
		t.Fatalf("expected solution with cost 18, found solution with cost %f", cost)
	}
	solution, cost, _ = search.Search(start, goal, 2.0)
	if cost != 28 {
		fmt.Println(world.SolutionString(solution))
		t.Fatalf("expected solution with cost 28, found solution with cost %f", cost)
	}
}

func TestSearch_CustomHeuristic(t *testing.T) {
	_, start, goal := newWorld(`
		@...*
	`)
	count := 0
	search.Search(start, goal, func(state search.State, other *search.State) float64 {
		count++
		return 0
	})
	if count != 5 {
		t.Fatalf("expected custom heuristic to be called 5 times, was called %d times", count)
	}
}

func BenchmarkSearch(b *testing.B) {
	rand.Seed(0)
	worldSize := 1024

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
	fmt.Printf("Benchmarking pathfinding on %dx%d world...\n", worldSize, worldSize)
	var solution []search.StateTransition
	var cost float64
	for i := 0; i < b.N; i++ {
		solution, cost, _ = search.Search(start, goal)
	}
	fmt.Printf("Shortest path found: %d steps long costing %f\n", len(solution), cost)
}
