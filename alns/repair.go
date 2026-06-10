package alns

import (
	"math"
	"math/rand"
	"sort"

	"github.com/logistics/vrptw-engine/model"
	"github.com/logistics/vrptw-engine/constraint"
)

type RepairOperator int

const (
	RepairGreedy RepairOperator = iota
	RepairRegret
	RepairCount
)

type RepairFunc func(sol *model.Solution, removed []int, nodes []*model.Node, mtx *model.Matrix, vehicles []*model.Vehicle, depots []*model.Node, rng *rand.Rand)

var RepairOps = [RepairCount]RepairFunc{
	RepairGreedy:  GreedyInsert,
	RepairRegret:  RegretInsert,
}

type insertPos struct {
	routeIdx int
	pos      int
	cost     float64
}

func GreedyInsert(sol *model.Solution, removed []int, nodes []*model.Node, mtx *model.Matrix, vehicles []*model.Vehicle, depots []*model.Node, rng *rand.Rand) {
	filtered := make([]int, 0, len(removed))
	for _, r := range removed {
		if !nodes[r].IsDepot {
			filtered = append(filtered, r)
		}
	}
	removed = filtered

	for len(removed) > 0 {
		bestNode := -1
		bestInsert := insertPos{routeIdx: -1, pos: -1, cost: math.MaxFloat64}

		for _, nodeIdx := range removed {
			ins := findBestInsert(sol, nodeIdx, nodes, mtx)
			if ins.cost < bestInsert.cost {
				bestInsert = ins
				bestNode = nodeIdx
			}
		}

		if bestInsert.routeIdx == -1 {
			added := tryAddNewRoute(sol, bestNode, nodes, mtx, vehicles, depots)
			if added {
				removed = removeNodeFromList(removed, bestNode)
				continue
			}
			sol.Unserved = append(sol.Unserved, removed...)
			return
		}

		sol.Routes[bestInsert.routeIdx].Nodes = insertAt(
			sol.Routes[bestInsert.routeIdx].Nodes, bestInsert.pos, bestNode,
		)
		removed = removeNodeFromList(removed, bestNode)
	}
}

func RegretInsert(sol *model.Solution, removed []int, nodes []*model.Node, mtx *model.Matrix, vehicles []*model.Vehicle, depots []*model.Node, rng *rand.Rand) {
	filtered := make([]int, 0, len(removed))
	for _, r := range removed {
		if !nodes[r].IsDepot {
			filtered = append(filtered, r)
		}
	}
	removed = filtered

	for len(removed) > 0 {
		bestNode := -1
		bestRegret := -1.0
		bestInsertForNode := insertPos{routeIdx: -1, pos: -1, cost: math.MaxFloat64}

		for _, nodeIdx := range removed {
			inserts := findAllInserts(sol, nodeIdx, nodes, mtx)
			regret := computeRegret(inserts)

			if regret > bestRegret || (regret == bestRegret && len(inserts) > 0 && inserts[0].cost < bestInsertForNode.cost) {
				bestRegret = regret
				bestNode = nodeIdx
				if len(inserts) > 0 {
					bestInsertForNode = inserts[0]
				}
			}
		}

		if bestInsertForNode.routeIdx == -1 {
			added := tryAddNewRoute(sol, bestNode, nodes, mtx, vehicles, depots)
			if added {
				removed = removeNodeFromList(removed, bestNode)
				continue
			}
			sol.Unserved = append(sol.Unserved, removed...)
			return
		}

		sol.Routes[bestInsertForNode.routeIdx].Nodes = insertAt(
			sol.Routes[bestInsertForNode.routeIdx].Nodes, bestInsertForNode.pos, bestNode,
		)
		removed = removeNodeFromList(removed, bestNode)
	}
}

func findBestInsert(sol *model.Solution, nodeIdx int, nodes []*model.Node, mtx *model.Matrix) insertPos {
	best := insertPos{routeIdx: -1, pos: -1, cost: math.MaxFloat64}

	for ri, r := range sol.Routes {
		for pos := 0; pos <= len(r.Nodes); pos++ {
			cost := constraint.InsertionCost(r, pos, nodeIdx, nodes, mtx)
			if cost < best.cost {
				best = insertPos{routeIdx: ri, pos: pos, cost: cost}
			}
		}
	}

	return best
}

func findAllInserts(sol *model.Solution, nodeIdx int, nodes []*model.Node, mtx *model.Matrix) []insertPos {
	var inserts []insertPos

	for ri, r := range sol.Routes {
		for pos := 0; pos <= len(r.Nodes); pos++ {
			cost := constraint.InsertionCost(r, pos, nodeIdx, nodes, mtx)
			inserts = append(inserts, insertPos{routeIdx: ri, pos: pos, cost: cost})
		}
	}

	sort.Slice(inserts, func(i, j int) bool {
		return inserts[i].cost < inserts[j].cost
	})

	return inserts
}

func computeRegret(inserts []insertPos) float64 {
	if len(inserts) == 0 {
		return math.MaxFloat64
	}
	if len(inserts) == 1 {
		return math.MaxFloat64 / 2
	}

	best := inserts[0].cost
	secondBest := inserts[1].cost

	for i := 2; i < len(inserts) && i < 5; i++ {
		if inserts[i].cost < secondBest {
			secondBest = inserts[i].cost
		}
	}

	return secondBest - best
}

func tryAddNewRoute(sol *model.Solution, nodeIdx int, nodes []*model.Node, mtx *model.Matrix, vehicles []*model.Vehicle, depots []*model.Node) bool {
	if nodeIdx < 0 {
		return false
	}

	for _, v := range vehicles {
		depotExists := false
		for _, r := range sol.Routes {
			if r.Vehicle.DepotID == v.DepotID {
				depotExists = true
				break
			}
		}

		if !depotExists {
			route := &model.Route{
				Vehicle: v,
				Nodes:   []int{nodeIdx},
			}
			constraint.EvaluateRoute(route, nodes, mtx)
			if route.LoadW <= v.CapWeight && route.LoadV <= v.CapVolume {
				sol.Routes = append(sol.Routes, route)
				return true
			}
		}
	}

	for _, v := range vehicles {
		route := &model.Route{
			Vehicle: v,
			Nodes:   []int{nodeIdx},
		}
		constraint.EvaluateRoute(route, nodes, mtx)
		if route.LoadW <= v.CapWeight && route.LoadV <= v.CapVolume {
			sol.Routes = append(sol.Routes, route)
			return true
		}
	}

	return false
}

func insertAt(slice []int, pos, val int) []int {
	result := make([]int, 0, len(slice)+1)
	result = append(result, slice[:pos]...)
	result = append(result, val)
	result = append(result, slice[pos:]...)
	return result
}

func removeNodeFromList(list []int, node int) []int {
	for i, n := range list {
		if n == node {
			return append(list[:i], list[i+1:]...)
		}
	}
	return list
}
