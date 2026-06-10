package alns

import (
	"math"
	"math/rand"
	"sort"
	"sync"

	"github.com/logistics/vrptw-engine/constraint"
	"github.com/logistics/vrptw-engine/model"
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

var insertPosPool = sync.Pool{
	New: func() interface{} {
		buf := make([]insertPos, 0, 512)
		return &buf
	},
}

func getInsertPosBuf() *[]insertPos {
	return insertPosPool.Get().(*[]insertPos)
}

func putInsertPosBuf(buf *[]insertPos) {
	*buf = (*buf)[:0]
	insertPosPool.Put(buf)
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
			buf := getInsertPosBuf()
			inserts := findAllInsertsBuf(sol, nodeIdx, nodes, mtx, buf)
			regret := computeRegret(inserts)

			if regret > bestRegret || (regret == bestRegret && len(inserts) > 0 && inserts[0].cost < bestInsertForNode.cost) {
				bestRegret = regret
				bestNode = nodeIdx
				if len(inserts) > 0 {
					bestInsertForNode = inserts[0]
				}
			}

			putInsertPosBuf(buf)
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
		if r.LoadW+nodes[nodeIdx].Demand > r.Vehicle.CapWeight {
			continue
		}
		if r.LoadV+nodes[nodeIdx].Volume > r.Vehicle.CapVolume {
			continue
		}
		for pos := 0; pos <= len(r.Nodes); pos++ {
			cost := constraint.InsertionCostFast(r, pos, nodeIdx, nodes, mtx)
			if cost < best.cost {
				best = insertPos{routeIdx: ri, pos: pos, cost: cost}
			}
		}
	}

	return best
}

func findAllInsertsBuf(sol *model.Solution, nodeIdx int, nodes []*model.Node, mtx *model.Matrix, buf *[]insertPos) []insertPos {
	inserts := (*buf)[:0]

	for ri, r := range sol.Routes {
		if r.LoadW+nodes[nodeIdx].Demand > r.Vehicle.CapWeight {
			continue
		}
		if r.LoadV+nodes[nodeIdx].Volume > r.Vehicle.CapVolume {
			continue
		}
		for pos := 0; pos <= len(r.Nodes); pos++ {
			cost := constraint.InsertionCostFast(r, pos, nodeIdx, nodes, mtx)
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
	result := make([]int, len(slice)+1)
	copy(result, slice[:pos])
	result[pos] = val
	copy(result[pos+1:], slice[pos:])
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
