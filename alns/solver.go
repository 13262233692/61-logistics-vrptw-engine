package alns

import (
	"math"
	"math/rand"
	"sync"
	"time"

	"github.com/logistics/vrptw-engine/constraint"
	"github.com/logistics/vrptw-engine/model"
)

const (
	SegmentSize   = 50
	WeightDecay   = 0.8
	ScoreNewBest  = 10.0
	ScoreBetter   = 5.0
	ScoreAccepted = 1.0
)

type ALNSConfig struct {
	MaxIter   int
	MaxTimeMs int64
	StartTemp float64
	CoolRate  float64
	MinRemove int
	MaxRemove int
	Seed      int64
}

func DefaultConfig() ALNSConfig {
	return ALNSConfig{
		MaxIter:   50000,
		MaxTimeMs: 60000,
		StartTemp: 50.0,
		CoolRate:  0.9997,
		MinRemove: 3,
		MaxRemove: 30,
		Seed:      time.Now().UnixNano(),
	}
}

type routeSnapshot struct {
	nodes       []int
	dist        float64
	time        float64
	loadW       float64
	loadV       float64
	loadFrozen  float64
	loadChilled float64
}

type solutionSnapshot struct {
	routes      []routeSnapshot
	routeCount  int
	unserved    []int
	totalDist   float64
	totalTime   float64
	feasible    bool
}

var snapshotPool = sync.Pool{
	New: func() interface{} {
		rs := make([]routeSnapshot, 0, 128)
		return &solutionSnapshot{
			routes: rs,
		}
	},
}

func (s *ALNSSolver) takeSnapshot(sol *model.Solution) *solutionSnapshot {
	snap := snapshotPool.Get().(*solutionSnapshot)
	snap.routeCount = len(sol.Routes)
	snap.totalDist = sol.TotalDist
	snap.totalTime = sol.TotalTime
	snap.feasible = sol.Feasible

	if cap(snap.routes) < len(sol.Routes) {
		snap.routes = make([]routeSnapshot, len(sol.Routes))
	} else {
		snap.routes = snap.routes[:len(sol.Routes)]
	}

	for i, r := range sol.Routes {
		nodesCopy := make([]int, len(r.Nodes))
		copy(nodesCopy, r.Nodes)
		snap.routes[i] = routeSnapshot{
			nodes:       nodesCopy,
			dist:        r.Dist,
			time:        r.Time,
			loadW:       r.LoadW,
			loadV:       r.LoadV,
			loadFrozen:  r.LoadFrozen,
			loadChilled: r.LoadChilled,
		}
	}

	if cap(snap.unserved) < len(sol.Unserved) {
		snap.unserved = make([]int, len(sol.Unserved))
	} else {
		snap.unserved = snap.unserved[:len(sol.Unserved)]
	}
	copy(snap.unserved, sol.Unserved)

	return snap
}

func (s *ALNSSolver) restoreSnapshot(sol *model.Solution, snap *solutionSnapshot) {
	for i := 0; i < snap.routeCount && i < len(sol.Routes); i++ {
		rs := &snap.routes[i]
		r := sol.Routes[i]
		if cap(r.Nodes) >= len(rs.nodes) {
			r.Nodes = r.Nodes[:len(rs.nodes)]
			copy(r.Nodes, rs.nodes)
		} else {
			r.Nodes = make([]int, len(rs.nodes))
			copy(r.Nodes, rs.nodes)
		}
		r.Dist = rs.dist
		r.Time = rs.time
		r.LoadW = rs.loadW
		r.LoadV = rs.loadV
		r.LoadFrozen = rs.loadFrozen
		r.LoadChilled = rs.loadChilled
	}

	sol.Routes = sol.Routes[:snap.routeCount]
	sol.TotalDist = snap.totalDist
	sol.TotalTime = snap.totalTime
	sol.Feasible = snap.feasible

	if cap(sol.Unserved) >= len(snap.unserved) {
		sol.Unserved = sol.Unserved[:len(snap.unserved)]
		copy(sol.Unserved, snap.unserved)
	} else {
		sol.Unserved = make([]int, len(snap.unserved))
		copy(sol.Unserved, snap.unserved)
	}

	s.releaseSnapshot(snap)
}

func (s *ALNSSolver) releaseSnapshot(snap *solutionSnapshot) {
	snap.routes = snap.routes[:0]
	snap.unserved = snap.unserved[:0]
	snapshotPool.Put(snap)
}

func (s *ALNSSolver) findDirtyRoutes(sol *model.Solution, snap *solutionSnapshot) []int {
	dirty := make([]int, 0, 8)
	minLen := len(snap.routes)
	if len(sol.Routes) < minLen {
		minLen = len(sol.Routes)
	}
	for i := 0; i < minLen; i++ {
		rs := &snap.routes[i]
		r := sol.Routes[i]
		if len(r.Nodes) != len(rs.nodes) {
			dirty = append(dirty, i)
			continue
		}
		for j := range r.Nodes {
			if r.Nodes[j] != rs.nodes[j] {
				dirty = append(dirty, i)
				break
			}
		}
	}
	for i := minLen; i < len(sol.Routes); i++ {
		dirty = append(dirty, i)
	}
	return dirty
}

func (s *ALNSSolver) evaluateDelta(sol *model.Solution, snap *solutionSnapshot) float64 {
	dirty := s.findDirtyRoutes(sol, snap)

	totalDist := snap.totalDist
	totalPenalty := 0.0
	sol.Feasible = true

	for _, idx := range dirty {
		if idx < len(snap.routes) {
			totalDist -= snap.routes[idx].dist
		}
	}

	for _, idx := range dirty {
		if idx < len(sol.Routes) {
			r := sol.Routes[idx]
			d, _, p, f := constraint.EvaluateRoute(r, s.Nodes, s.Matrix)
			totalDist += d
			totalPenalty += p
			if !f {
				sol.Feasible = false
			}
		}
	}

	totalPenalty += constraint.PenaltyUnserved * float64(len(sol.Unserved))
	if len(sol.Unserved) > 0 {
		sol.Feasible = false
	}

	sol.TotalDist = totalDist
	return totalDist + totalPenalty
}

type ALNSSolver struct {
	Config   ALNSConfig
	Nodes    []*model.Node
	Vehicles []*model.Vehicle
	Matrix   *model.Matrix
	Depots   []*model.Node
	Rng      *rand.Rand

	destroyWeights []float64
	repairWeights  []float64
	destroyScores  []float64
	repairScores   []float64
	destroyCounts  []int
	repairCounts   []int

	bestSol    *model.Solution
	bestCost   float64
	currSol    *model.Solution
	currCost   float64
	iterations int
}

func NewALNSSolver(prob *model.Problem, cfg ALNSConfig) *ALNSSolver {
	rng := rand.New(rand.NewSource(cfg.Seed))
	dw := make([]float64, DestroyCount)
	rw := make([]float64, RepairCount)
	ds := make([]float64, DestroyCount)
	rs := make([]float64, RepairCount)
	dc := make([]int, DestroyCount)
	rc := make([]int, RepairCount)

	for i := range dw {
		dw[i] = 1.0
	}
	for i := range rw {
		rw[i] = 1.0
	}

	return &ALNSSolver{
		Config:         cfg,
		Nodes:          prob.Nodes,
		Vehicles:       prob.Vehicles,
		Matrix:         prob.Matrix,
		Depots:         prob.Depots,
		Rng:            rng,
		destroyWeights: dw,
		repairWeights:  rw,
		destroyScores:  ds,
		repairScores:   rs,
		destroyCounts:  dc,
		repairCounts:   rc,
	}
}

func (s *ALNSSolver) Solve() *model.Solution {
	s.currSol = s.initialSolution()
	s.currCost = constraint.EvaluateSolution(s.currSol, s.Nodes, s.Matrix)
	s.bestSol = s.deepcopySolution(s.currSol)
	s.bestCost = s.currCost

	temperature := s.Config.StartTemp
	startTime := time.Now()

	for iter := 0; iter < s.Config.MaxIter; iter++ {
		if time.Since(startTime).Milliseconds() > s.Config.MaxTimeMs {
			break
		}

		s.iterations = iter + 1

		q := s.Rng.Intn(s.Config.MaxRemove-s.Config.MinRemove+1) + s.Config.MinRemove
		served := countServed(s.currSol)
		if q > len(s.currSol.Unserved)+served {
			q = len(s.currSol.Unserved) + served
		}

		snap := s.takeSnapshot(s.currSol)

		dIdx := s.selectOperator(s.destroyWeights)
		rIdx := s.selectOperator(s.repairWeights)

		destroyed := DestroyOps[dIdx](s.currSol, s.Nodes, s.Matrix, s.Rng, q)
		allRemoved := make([]int, 0, len(s.currSol.Unserved)+len(destroyed))
		allRemoved = append(allRemoved, s.currSol.Unserved...)
		allRemoved = append(allRemoved, destroyed...)

		RepairOps[rIdx](s.currSol, allRemoved, s.Nodes, s.Matrix, s.Vehicles, s.Depots, s.Rng)

		newCost := s.evaluateDelta(s.currSol, snap)

		score := 0.0
		accepted := false

		if newCost < s.bestCost {
			score = ScoreNewBest
			s.bestSol = s.deepcopySolution(s.currSol)
			s.bestCost = newCost
			accepted = true
		} else if newCost < s.currCost {
			score = ScoreBetter
			accepted = true
		} else if acceptWorse(newCost, s.currCost, temperature, s.Rng) {
			score = ScoreAccepted
			accepted = true
		}

		if accepted {
			s.currCost = newCost
			s.releaseSnapshot(snap)
		} else {
			s.restoreSnapshot(s.currSol, snap)
		}

		s.destroyScores[dIdx] += score
		s.repairScores[rIdx] += score
		s.destroyCounts[dIdx]++
		s.repairCounts[rIdx]++

		if (iter+1)%SegmentSize == 0 {
			s.updateWeights()
		}

		temperature *= s.Config.CoolRate
		if temperature < 0.01 {
			temperature = 0.01
		}
	}

	cleanupEmptyRoutes(s.bestSol)
	constraint.EvaluateSolution(s.bestSol, s.Nodes, s.Matrix)
	return s.bestSol
}

func (s *ALNSSolver) Iterations() int {
	return s.iterations
}

func (s *ALNSSolver) selectOperator(weights []float64) int {
	total := 0.0
	for _, w := range weights {
		total += w
	}

	r := s.Rng.Float64() * total
	cumulative := 0.0
	for i, w := range weights {
		cumulative += w
		if r <= cumulative {
			return i
		}
	}
	return len(weights) - 1
}

func (s *ALNSSolver) updateWeights() {
	for i := 0; i < int(DestroyCount); i++ {
		if s.destroyCounts[i] > 0 {
			avg := s.destroyScores[i] / float64(s.destroyCounts[i])
			s.destroyWeights[i] = WeightDecay*s.destroyWeights[i] + (1-WeightDecay)*avg
			if s.destroyWeights[i] < 0.1 {
				s.destroyWeights[i] = 0.1
			}
		}
		s.destroyScores[i] = 0
		s.destroyCounts[i] = 0
	}

	for i := 0; i < int(RepairCount); i++ {
		if s.repairCounts[i] > 0 {
			avg := s.repairScores[i] / float64(s.repairCounts[i])
			s.repairWeights[i] = WeightDecay*s.repairWeights[i] + (1-WeightDecay)*avg
			if s.repairWeights[i] < 0.1 {
				s.repairWeights[i] = 0.1
			}
		}
		s.repairScores[i] = 0
		s.repairCounts[i] = 0
	}
}

func acceptWorse(newCost, currCost, temp float64, rng *rand.Rand) bool {
	if temp <= 0 {
		return false
	}
	delta := newCost - currCost
	if delta <= 0 {
		return true
	}
	prob := math.Exp(-delta / temp)
	return rng.Float64() < prob
}

func (s *ALNSSolver) initialSolution() *model.Solution {
	sol := &model.Solution{}

	customerIDs := make([]int, 0)
	for _, n := range s.Nodes {
		if !n.IsDepot {
			customerIDs = append(customerIDs, n.ID)
		}
	}

	s.Rng.Shuffle(len(customerIDs), func(i, j int) {
		customerIDs[i], customerIDs[j] = customerIDs[j], customerIDs[i]
	})

	for _, v := range s.Vehicles {
		for k := 0; k < v.Count; k++ {
			sol.Routes = append(sol.Routes, &model.Route{
				Vehicle: v,
				Nodes:   []int{},
			})
		}
	}

	if len(sol.Routes) == 0 {
		for _, v := range s.Vehicles {
			sol.Routes = append(sol.Routes, &model.Route{
				Vehicle: v,
				Nodes:   []int{},
			})
		}
	}

	RegretInsert(sol, customerIDs, s.Nodes, s.Matrix, s.Vehicles, s.Depots, s.Rng)

	return sol
}

func (s *ALNSSolver) deepcopySolution(sol *model.Solution) *model.Solution {
	newSol := &model.Solution{
		TotalDist: sol.TotalDist,
		TotalTime: sol.TotalTime,
		Feasible:  sol.Feasible,
	}

	newSol.Routes = make([]*model.Route, len(sol.Routes))
	for i, r := range sol.Routes {
		nodes := make([]int, len(r.Nodes))
		copy(nodes, r.Nodes)
		newSol.Routes[i] = &model.Route{
			Vehicle:     r.Vehicle,
			Nodes:       nodes,
			LoadW:       r.LoadW,
			LoadV:       r.LoadV,
			LoadFrozen:  r.LoadFrozen,
			LoadChilled: r.LoadChilled,
			Dist:        r.Dist,
			Time:        r.Time,
		}
	}

	newSol.Unserved = make([]int, len(sol.Unserved))
	copy(newSol.Unserved, sol.Unserved)

	return newSol
}

func countServed(sol *model.Solution) int {
	count := 0
	for _, r := range sol.Routes {
		count += len(r.Nodes)
	}
	return count
}

func cleanupEmptyRoutes(sol *model.Solution) {
	routes := make([]*model.Route, 0, len(sol.Routes))
	for _, r := range sol.Routes {
		if len(r.Nodes) > 0 {
			routes = append(routes, r)
		}
	}
	sol.Routes = routes
}
