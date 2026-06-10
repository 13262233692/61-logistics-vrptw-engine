package constraint

import (
	"math"

	"github.com/logistics/vrptw-engine/model"
)

const (
	PenaltyTW      = 10000.0
	PenaltyCap     = 10000.0
	PenaltyUnserved = 5000.0
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
		return 0, 0, 0, true
	}

	depotIdx := route.Vehicle.DepotID
	totalDist := 0.0
	totalTime := 0.0
	loadW := 0.0
	loadV := 0.0
	penalty = 0.0
	feasible = true

	prev := depotIdx
	currentTime := nodes[depotIdx].TW.Earliest

	for _, ni := range route.Nodes {
		if nodes[ni].IsDepot {
			continue
		}

		totalDist += mtx.Dist[prev][ni]
		travelTime := mtx.Time[prev][ni]
		currentTime += travelTime

		node := nodes[ni]
		loadW += node.Demand
		loadV += node.Volume

		if currentTime < node.TW.Earliest {
			currentTime = node.TW.Earliest
		}
		if currentTime > node.TW.Latest {
			twViolation := currentTime - node.TW.Latest
			penalty += PenaltyTW * twViolation
			feasible = false
		}

		currentTime += node.Service
		prev = ni
	}

	totalDist += mtx.Dist[prev][depotIdx]
	totalTime = currentTime + mtx.Time[prev][depotIdx] - nodes[depotIdx].TW.Earliest

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

func InsertionCost(route *model.Route, pos, nodeIdx int, nodes []*model.Node, mtx *model.Matrix) float64 {
	newNodes := make([]int, 0, len(route.Nodes)+1)
	newNodes = append(newNodes, route.Nodes[:pos]...)
	newNodes = append(newNodes, nodeIdx)
	newNodes = append(newNodes, route.Nodes[pos:]...)

	tmpRoute := &model.Route{
		Vehicle: route.Vehicle,
		Nodes:   newNodes,
	}

	dist, _, penalty, _ := EvaluateRoute(tmpRoute, nodes, mtx)
	oldDist := route.Dist

	return (dist - oldDist) + penalty
}

func CanInsert(route *model.Route, pos, nodeIdx int, nodes []*model.Node, mtx *model.Matrix) bool {
	newNodes := make([]int, 0, len(route.Nodes)+1)
	newNodes = append(newNodes, route.Nodes[:pos]...)
	newNodes = append(newNodes, nodeIdx)
	newNodes = append(newNodes, route.Nodes[pos:]...)

	loadW := route.LoadW + nodes[nodeIdx].Demand
	loadV := route.LoadV + nodes[nodeIdx].Volume
	if loadW > route.Vehicle.CapWeight || loadV > route.Vehicle.CapVolume {
		return false
	}

	tmpRoute := &model.Route{
		Vehicle: route.Vehicle,
		Nodes:   newNodes,
		LoadW:   loadW,
		LoadV:   loadV,
	}

	_, _, _, feasible := EvaluateRoute(tmpRoute, nodes, mtx)
	return feasible
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
