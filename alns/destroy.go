package alns

import (
	"math"
	"math/rand"
	"sort"

	"github.com/logistics/vrptw-engine/model"
)

type DestroyOperator int

const (
	DestroyShaw DestroyOperator = iota
	DestroyWorst
	DestroyRandom
	DestroyCount
)

type DestroyFunc func(sol *model.Solution, nodes []*model.Node, mtx *model.Matrix, rng *rand.Rand, q int) []int

var DestroyOps = [DestroyCount]DestroyFunc{
	DestroyShaw:   ShawRemove,
	DestroyWorst:  WorstRemove,
	DestroyRandom: RandomRemove,
}

func ShawRemove(sol *model.Solution, nodes []*model.Node, mtx *model.Matrix, rng *rand.Rand, q int) []int {
	customers := collectCustomers(sol, nodes)
	if len(customers) == 0 {
		return nil
	}
	if q > len(customers) {
		q = len(customers)
	}

	seed := customers[rng.Intn(len(customers))]
	removed := []int{seed}
	removeFromSolution(sol, seed)

	remaining := make([]int, 0, len(customers)-1)
	for _, c := range customers {
		if c != seed {
			remaining = append(remaining, c)
		}
	}

	for len(removed) < q && len(remaining) > 0 {
		rIdx := removed[rng.Intn(len(removed))]
		bestIdx := 0
		bestSim := -1.0

		for i, c := range remaining {
			sim := shawSimilarity(rIdx, c, nodes, mtx)
			if sim > bestSim {
				bestSim = sim
				bestIdx = i
			}
		}

		chosen := remaining[bestIdx]
		removed = append(removed, chosen)
		removeFromSolution(sol, chosen)
		remaining = append(remaining[:bestIdx], remaining[bestIdx+1:]...)
	}

	return removed
}

func shawSimilarity(i, j int, nodes []*model.Node, mtx *model.Matrix) float64 {
	di := mtx.Dist[i][j]
	ti := math.Abs(nodes[i].TW.Earliest - nodes[j].TW.Earliest)
	diNorm := di / maxRouteDist(mtx)
	tiNorm := ti / maxTWWidth(nodes)

	return -(diNorm + tiNorm)
}

func maxRouteDist(mtx *model.Matrix) float64 {
	mx := 0.0
	for i := 0; i < mtx.N; i++ {
		for j := 0; j < mtx.N; j++ {
			if mtx.Dist[i][j] > mx {
				mx = mtx.Dist[i][j]
			}
		}
	}
	if mx == 0 {
		return 1
	}
	return mx
}

func maxTWWidth(nodes []*model.Node) float64 {
	mx := 0.0
	for _, n := range nodes {
		w := n.TW.Latest - n.TW.Earliest
		if w > mx {
			mx = w
		}
	}
	if mx == 0 {
		return 1
	}
	return mx
}

func WorstRemove(sol *model.Solution, nodes []*model.Node, mtx *model.Matrix, rng *rand.Rand, q int) []int {
	customers := collectCustomers(sol, nodes)
	if len(customers) == 0 {
		return nil
	}
	if q > len(customers) {
		q = len(customers)
	}

	removed := make([]int, 0, q)

	for len(removed) < q {
		customers = collectCustomers(sol, nodes)
		if len(customers) == 0 {
			break
		}

		type costEntry struct {
			node int
			cost float64
		}
		costs := make([]costEntry, 0, len(customers))

		for _, c := range customers {
			route, pos := findNodeInSolution(sol, c)
			if route == nil {
				continue
			}
			cost := removalGain(route, pos, nodes, mtx)
			costs = append(costs, costEntry{node: c, cost: cost})
		}

		sort.Slice(costs, func(i, j int) bool {
			return costs[i].cost > costs[j].cost
		})

		p := rng.Intn(min(5, len(costs)))
		chosen := costs[p].node
		removed = append(removed, chosen)
		removeFromSolution(sol, chosen)
	}

	return removed
}

func removalGain(route *model.Route, pos int, nodes []*model.Node, mtx *model.Matrix) float64 {
	depotIdx := route.Vehicle.DepotID
	prev := depotIdx
	for p := pos - 1; p >= 0; p-- {
		if !nodes[route.Nodes[p]].IsDepot {
			prev = route.Nodes[p]
			break
		}
	}
	next := depotIdx
	for n := pos + 1; n < len(route.Nodes); n++ {
		if !nodes[route.Nodes[n]].IsDepot {
			next = route.Nodes[n]
			break
		}
	}

	withNode := mtx.Dist[prev][route.Nodes[pos]] + mtx.Dist[route.Nodes[pos]][next]
	withoutNode := mtx.Dist[prev][next]

	return withNode - withoutNode
}

func RandomRemove(sol *model.Solution, nodes []*model.Node, mtx *model.Matrix, rng *rand.Rand, q int) []int {
	customers := collectCustomers(sol, nodes)
	if len(customers) == 0 {
		return nil
	}
	if q > len(customers) {
		q = len(customers)
	}

	rng.Shuffle(len(customers), func(i, j int) {
		customers[i], customers[j] = customers[j], customers[i]
	})

	removed := customers[:q]
	for _, c := range removed {
		removeFromSolution(sol, c)
	}
	return removed
}

func collectServed(sol *model.Solution) []int {
	served := make([]int, 0)
	for _, r := range sol.Routes {
		for _, n := range r.Nodes {
			served = append(served, n)
		}
	}
	return served
}

func collectCustomers(sol *model.Solution, nodes []*model.Node) []int {
	served := collectServed(sol)
	customers := make([]int, 0, len(served))
	for _, s := range served {
		if !nodes[s].IsDepot {
			customers = append(customers, s)
		}
	}
	return customers
}

func findNodeInSolution(sol *model.Solution, nodeID int) (*model.Route, int) {
	for _, r := range sol.Routes {
		for i, n := range r.Nodes {
			if n == nodeID {
				return r, i
			}
		}
	}
	return nil, -1
}

func removeFromSolution(sol *model.Solution, nodeID int) {
	for _, r := range sol.Routes {
		for i, n := range r.Nodes {
			if n == nodeID {
				r.Nodes = append(r.Nodes[:i], r.Nodes[i+1:]...)
				return
			}
		}
	}
}

func min(a, b int) int {
	if a < b {
		return a
	}
	return b
}
