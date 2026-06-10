package constraint

import (
	"math"

	"github.com/logistics/vrptw-engine/model"
)

const (
	PenaltyTW          = 10000.0
	PenaltyCap         = 10000.0
	PenaltyZoneCap     = 15000.0
	PenaltyUnserved    = 5000.0
)

func BuildProblem(nodes []*model.Node, vehicles []*Vehicle, mtx *model.Matrix) *model.Problem {
	depots := make([]*model.Node, 0)
	customers := make([]*model.Node, 0)
	for _, n := range nodes {
		if n.IsDepot {
			depots = append(depots, n)
		} else {
			customers = append(customers, n)
		}
	}
	return &model.Problem{
		Nodes:     nodes,
		Vehicles:  vehicles,
		Matrix:    mtx,
		Depots:    depots,
		Customers: customers,
	}
}

type Vehicle = model.Vehicle

func EvaluateRoute(route *model.Route, nodes []*model.Node, mtx *model.Matrix) (dist, duration, penalty float64, feasible bool) {
	if len(route.Nodes) == 0 {
		route.Dist = 0
		route.Time = 0
		route.LoadW = 0
		route.LoadV = 0
		route.LoadFrozen = 0
		route.LoadChilled = 0
		return 0, 0, 0, true
	}

	depotIdx := route.Vehicle.DepotID
	totalDist := 0.0
	loadW := 0.0
	loadV := 0.0
	loadFrozen := 0.0
	loadChilled := 0.0
	penalty = 0.0
	feasible = true

	prev := depotIdx
	currentTime := nodes[depotIdx].TW.Earliest

	for _, ni := range route.Nodes {
		if nodes[ni].IsDepot {
			continue
		}

		totalDist += mtx.Dist[prev][ni]
		currentTime += mtx.Time[prev][ni]

		node := nodes[ni]
		loadW += node.Demand
		loadV += node.Volume

		if node.TempZone == model.TempZoneFrozen {
			loadFrozen += node.Volume
		} else {
			loadChilled += node.Volume
		}

		if route.Vehicle.CapFrozen > 0 && loadFrozen > route.Vehicle.CapFrozen {
			penalty += PenaltyZoneCap * (loadFrozen - route.Vehicle.CapFrozen)
			feasible = false
		}
		if route.Vehicle.CapChilled > 0 && loadChilled > route.Vehicle.CapChilled {
			penalty += PenaltyZoneCap * (loadChilled - route.Vehicle.CapChilled)
			feasible = false
		}

		if currentTime < node.TW.Earliest {
			currentTime = node.TW.Earliest
		}
		if currentTime > node.TW.Latest {
			penalty += PenaltyTW * (currentTime - node.TW.Latest)
			feasible = false
		}

		currentTime += node.Service
		prev = ni
	}

	totalDist += mtx.Dist[prev][depotIdx]
	totalTime := currentTime + mtx.Time[prev][depotIdx] - nodes[depotIdx].TW.Earliest

	if loadW > route.Vehicle.CapWeight {
		penalty += PenaltyCap * (loadW - route.Vehicle.CapWeight)
		feasible = false
	}
	if loadV > route.Vehicle.CapVolume {
		penalty += PenaltyCap * (loadV - route.Vehicle.CapVolume)
		feasible = false
	}

	route.LoadW = loadW
	route.LoadV = loadV
	route.LoadFrozen = loadFrozen
	route.LoadChilled = loadChilled
	route.Dist = totalDist
	route.Time = totalTime

	return totalDist, totalTime, penalty, feasible
}

func EvaluateSolution(sol *model.Solution, nodes []*model.Node, mtx *model.Matrix) float64 {
	totalDist := 0.0
	totalTime := 0.0
	totalPenalty := 0.0
	sol.Feasible = true

	for _, r := range sol.Routes {
		d, dur, p, f := EvaluateRoute(r, nodes, mtx)
		totalDist += d
		totalTime += dur
		totalPenalty += p
		if !f {
			sol.Feasible = false
		}
	}

	totalPenalty += PenaltyUnserved * float64(len(sol.Unserved))
	if len(sol.Unserved) > 0 {
		sol.Feasible = false
	}

	sol.TotalDist = totalDist
	sol.TotalTime = totalTime
	return totalDist + totalPenalty
}

func InsertionCostFast(route *model.Route, pos, nodeIdx int, nodes []*model.Node, mtx *model.Matrix) float64 {
	depotIdx := route.Vehicle.DepotID
	node := nodes[nodeIdx]

	if route.LoadW+node.Demand > route.Vehicle.CapWeight {
		return math.MaxFloat64 / 4
	}
	if route.LoadV+node.Volume > route.Vehicle.CapVolume {
		return math.MaxFloat64 / 4
	}

	nodeIsFrozen := node.TempZone == model.TempZoneFrozen
	if nodeIsFrozen && route.Vehicle.CapFrozen > 0 && route.LoadFrozen+node.Volume > route.Vehicle.CapFrozen {
		return math.MaxFloat64 / 4
	}
	if !nodeIsFrozen && route.Vehicle.CapChilled > 0 && route.LoadChilled+node.Volume > route.Vehicle.CapChilled {
		return math.MaxFloat64 / 4
	}

	prev := depotIdx
	if pos > 0 {
		prev = route.Nodes[pos-1]
	}
	next := depotIdx
	if pos < len(route.Nodes) {
		next = route.Nodes[pos]
	}

	distDelta := mtx.Dist[prev][nodeIdx] + mtx.Dist[nodeIdx][next] - mtx.Dist[prev][next]

	currentTime := nodes[depotIdx].TW.Earliest
	prevNode := depotIdx
	penalty := 0.0
	runFrozen := 0.0
	runChilled := 0.0

	for i := 0; i < pos; i++ {
		ni := route.Nodes[i]
		nd := nodes[ni]
		currentTime += mtx.Time[prevNode][ni]
		if currentTime < nd.TW.Earliest {
			currentTime = nd.TW.Earliest
		}
		currentTime += nd.Service
		prevNode = ni
		if !nd.IsDepot {
			if nd.TempZone == model.TempZoneFrozen {
				runFrozen += nd.Volume
			} else {
				runChilled += nd.Volume
			}
		}
	}

	currentTime += mtx.Time[prevNode][nodeIdx]
	if currentTime < node.TW.Earliest {
		currentTime = node.TW.Earliest
	}
	if currentTime > node.TW.Latest {
		penalty += PenaltyTW * (currentTime - node.TW.Latest)
	}
	currentTime += node.Service
	prevNode = nodeIdx

	if nodeIsFrozen {
		runFrozen += node.Volume
	} else {
		runChilled += node.Volume
	}

	if route.Vehicle.CapFrozen > 0 && runFrozen > route.Vehicle.CapFrozen {
		penalty += PenaltyZoneCap * (runFrozen - route.Vehicle.CapFrozen)
	}
	if route.Vehicle.CapChilled > 0 && runChilled > route.Vehicle.CapChilled {
		penalty += PenaltyZoneCap * (runChilled - route.Vehicle.CapChilled)
	}

	for i := pos; i < len(route.Nodes); i++ {
		ni := route.Nodes[i]
		nd := nodes[ni]
		currentTime += mtx.Time[prevNode][ni]
		if currentTime < nd.TW.Earliest {
			currentTime = nd.TW.Earliest
		}
		if currentTime > nd.TW.Latest {
			penalty += PenaltyTW * (currentTime - nd.TW.Latest)
		}
		currentTime += nd.Service
		prevNode = ni
		if !nd.IsDepot {
			if nd.TempZone == model.TempZoneFrozen {
				runFrozen += nd.Volume
			} else {
				runChilled += nd.Volume
			}
		}
		if route.Vehicle.CapFrozen > 0 && runFrozen > route.Vehicle.CapFrozen {
			penalty += PenaltyZoneCap * (runFrozen - route.Vehicle.CapFrozen)
		}
		if route.Vehicle.CapChilled > 0 && runChilled > route.Vehicle.CapChilled {
			penalty += PenaltyZoneCap * (runChilled - route.Vehicle.CapChilled)
		}
	}

	return distDelta + penalty
}

func InsertionCost(route *model.Route, pos, nodeIdx int, nodes []*model.Node, mtx *model.Matrix) float64 {
	return InsertionCostFast(route, pos, nodeIdx, nodes, mtx)
}

func NearestDepot(nodeIdx int, nodes []*model.Node, depots []*model.Node, mtx *model.Matrix) int {
	bestDepot := depots[0].ID
	bestDist := math.MaxFloat64
	for _, d := range depots {
		if mtx.Dist[d.ID][nodeIdx] < bestDist {
			bestDist = mtx.Dist[d.ID][nodeIdx]
			bestDepot = d.ID
		}
	}
	return bestDepot
}
