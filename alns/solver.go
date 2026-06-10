package alns

import (
	"math"
	"math/rand"
	"time"

	"github.com/logistics/vrptw-engine/constraint"
	"github.com/logistics/vrptw-engine/model"
)

const (
	SegmentSize    = 50
	WeightDecay    = 0.8
	ScoreNewBest   = 10.0
	ScoreBetter    = 5.0
	ScoreAccepted  = 1.0
)

type ALNSConfig struct {
	MaxIter      int
	MaxTimeMs    int64
	StartTemp    float64
	CoolRate     float64
	MinRemove    int
	MaxRemove    int
	Seed         int64
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

type ALNSSolver struct {
	Config     ALNSConfig
	Nodes      []*model.Node
	Vehicles   []*model.Vehicle
	Matrix     *model.Matrix
	Depots     []*model.Node
	Rng        *rand.Rand

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
	s.bestSol = s.copySolution(s.currSol)
	s.bestCost = s.currCost

	temperature := s.Config.StartTemp
	startTime := time.Now()

	for iter := 0; iter < s.Config.MaxIter; iter++ {
		if time.Since(startTime).Milliseconds() > s.Config.MaxTimeMs {
			break
		}

		s.iterations = iter + 1

		q := s.Rng.Intn(s.Config.MaxRemove-s.Config.MinRemove+1) + s.Config.MinRemove
		if q > len(s.currSol.Unserved)+countServed(s.currSol) {
			q = len(s.currSol.Unserved) + countServed(s.currSol)
		}

		newSol := s.copySolution(s.currSol)

		dIdx := s.selectOperator(s.destroyWeights)
		rIdx := s.selectOperator(s.repairWeights)

		allRemoved := append([]int{}, newSol.Unserved...)
		destroyed := DestroyOps[dIdx](newSol, s.Nodes, s.Matrix, s.Rng, q)
		allRemoved = append(allRemoved, destroyed...)

		RepairOps[rIdx](newSol, allRemoved, s.Nodes, s.Matrix, s.Vehicles, s.Depots, s.Rng)

		newCost := constraint.EvaluateSolution(newSol, s.Nodes, s.Matrix)

		score := 0.0
		if newCost < s.bestCost {
			score = ScoreNewBest
			s.bestSol = s.copySolution(newSol)
			s.bestCost = newCost
		} else if newCost < s.currCost {
			score = ScoreBetter
		} else if acceptWorse(newCost, s.currCost, temperature, s.Rng) {
			score = ScoreAccepted
		}

		if newCost < s.currCost || acceptWorse(newCost, s.currCost, temperature, s.Rng) {
			s.currSol = newSol
			s.currCost = newCost
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

func (s *ALNSSolver) copySolution(sol *model.Solution) *model.Solution {
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
			Vehicle: r.Vehicle,
			Nodes:   nodes,
			LoadW:   r.LoadW,
			LoadV:   r.LoadV,
			Dist:    r.Dist,
			Time:    r.Time,
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
