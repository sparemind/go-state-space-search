package astar_test

import (
	"fmt"
	"github.com/sparemind/go-astar"
	"testing"
)

func assertSolutionCost(t *testing.T, expectedCost float64, worldString string, args ...position) {
	world, start, goal := newWorld(worldString)
	if len(args) > 0 {
		start.x, start.y = args[0].x, args[0].y
		if len(args) > 1 {
			goal.x, goal.y = args[1].x, args[1].y
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
}

func TestSearch_Search_NoSolution(t *testing.T) {
	assertSolutionCost(t, -1, `
		..#..
		@.#.*
		..#..
	`)
	assertSolutionCost(t, -1, `
		...#
		@.#*
		...#
	`)
	assertSolutionCost(t, -1, `
		#...
		@#.*
		#...
	`)
}

func TestSearch_EmptySolution(t *testing.T) {
	assertSolutionCost(t, 0, `
		111
		1@1
		111
	`, position{x: 1, y: 1}, position{x: 1, y: 1})
}

func TestSearch_StraightPath(t *testing.T) {
	assertSolutionCost(t, 6, `
		.@.....*.
	`)
	assertSolutionCost(t, 4, `
		.....
		@...*
		.....
	`)
	assertSolutionCost(t, 4, `
		.....
		*...@
		.....
	`)
}

func TestSearch_AvoidHighCost(t *testing.T) {
	assertSolutionCost(t, 6, `
		.....
		@.9.*
		.....
	`)
	assertSolutionCost(t, 12, `
		@.594.*
		..643..
		..28...
		.......
	`)
}

func TestSearch_TakeShortcut(t *testing.T) {
	assertSolutionCost(t, 7, `
		@.4.*
		..9..
		.....
	`)
	assertSolutionCost(t, 10, `
		@..23..*
		...789..
		...456..
		........
	`)
}

func TestSearch_LongPath(t *testing.T) {
	assertSolutionCost(t, 18, `
		@..2..2....2...
		.2.2..22.2...2.
		...........2...
		2.2.222222..222
		2.........2...*
	`)
}
