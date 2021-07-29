package astar_test

import (
	"github.com/sparemind/go-astar"
	"math/rand"
	"testing"
)

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
