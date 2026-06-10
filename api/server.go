package api

import (
	"encoding/json"
	"log"
	"net/http"
	"time"

	"github.com/logistics/vrptw-engine/alns"
	"github.com/logistics/vrptw-engine/constraint"
	"github.com/logistics/vrptw-engine/model"
	"github.com/logistics/vrptw-engine/osrm"
)

type Server struct {
	Port    string
	Fetcher *osrm.Fetcher
}

func NewServer(port string, osrmURL string) *Server {
	return &Server{
		Port:    port,
		Fetcher: osrm.NewFetcher(osrmURL, 8),
	}
}

func (s *Server) Start() error {
	mux := http.NewServeMux()
	mux.HandleFunc("/solve", s.handleSolve)
	mux.HandleFunc("/health", s.handleHealth)

	log.Printf("VRPTW Engine listening on %s", s.Port)
	return http.ListenAndServe(s.Port, mux)
}

func (s *Server) handleSolve(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		writeError(w, http.StatusMethodNotAllowed, "POST only")
		return
	}

	var req model.SolveRequest
	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		writeError(w, http.StatusBadRequest, "invalid JSON: "+err.Error())
		return
	}

	if len(req.Nodes) == 0 {
		writeError(w, http.StatusBadRequest, "no nodes provided")
		return
	}
	if len(req.Vehicles) == 0 {
		writeError(w, http.StatusBadRequest, "no vehicles provided")
		return
	}

	fetcher := s.Fetcher
	if req.OSRMURL != "" {
		fetcher = osrm.NewFetcher(req.OSRMURL, 8)
	}

	log.Printf("Fetching distance matrix for %d nodes...", len(req.Nodes))
	startMatrix := time.Now()
	mtx, err := fetcher.FetchMatrix(req.Nodes)
	if err != nil {
		log.Printf("Matrix fetch failed, using haversine fallback: %v", err)
		fallback := osrm.NewFetcher("", 1)
		mtx, err = fallback.FetchMatrix(req.Nodes)
		if err != nil {
			writeError(w, http.StatusInternalServerError, "matrix computation failed: "+err.Error())
			return
		}
	}
	log.Printf("Matrix fetched in %v", time.Since(startMatrix))

	prob := constraint.BuildProblem(req.Nodes, req.Vehicles, mtx)

	cfg := alns.DefaultConfig()
	cfg.MaxTimeMs = 60000
	cfg.MaxIter = 50000

	log.Printf("Starting ALNS solver (%d iterations, %dms timeout)...", cfg.MaxIter, cfg.MaxTimeMs)
	startSolve := time.Now()

	solver := alns.NewALNSSolver(prob, cfg)
	solution := solver.Solve()

	elapsed := time.Since(startSolve)
	log.Printf("ALNS solved in %v, cost=%.2f, feasible=%v, routes=%d", elapsed, solution.TotalDist, solution.Feasible, len(solution.Routes))

	resp := buildResponse(solution, prob.Nodes, solver.Iterations())

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(resp)
}

func (s *Server) handleHealth(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]string{"status": "ok"})
}

func buildResponse(sol *model.Solution, nodes []*model.Node, iterations int) *model.SolveResponse {
	resp := &model.SolveResponse{
		TotalDist:  sol.TotalDist,
		TotalTime:  sol.TotalTime,
		Feasible:   sol.Feasible,
		Unserved:   sol.Unserved,
		Iterations: iterations,
	}

	resp.Routes = make([]model.RouteResult, 0, len(sol.Routes))
	for _, r := range sol.Routes {
		if len(r.Nodes) == 0 {
			continue
		}
		seq := make([]int, 0, len(r.Nodes))
		for _, n := range r.Nodes {
			if !nodes[n].IsDepot {
				seq = append(seq, n)
			}
		}
		if len(seq) == 0 {
			continue
		}
		resp.Routes = append(resp.Routes, model.RouteResult{
			VehicleID:   r.Vehicle.ID,
			DepotID:     r.Vehicle.DepotID,
			Sequence:    seq,
			Distance:    r.Dist,
			Duration:    r.Time,
			LoadWeight:  r.LoadW,
			LoadVolume:  r.LoadV,
			LoadFrozen:  r.LoadFrozen,
			LoadChilled: r.LoadChilled,
		})
	}

	return resp
}

func writeError(w http.ResponseWriter, code int, msg string) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(code)
	json.NewEncoder(w).Encode(map[string]string{"error": msg})
}
