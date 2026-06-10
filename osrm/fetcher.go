package osrm

import (
	"encoding/json"
	"fmt"
	"io"
	"math"
	"net/http"
	"sync"
	"time"

	"github.com/logistics/vrptw-engine/model"
)

type osrmResponse struct {
	Code   string `json:"code"`
	Durations [][]float64 `json:"durations"`
	Distances [][]float64 `json:"distances"`
}

type Fetcher struct {
	BaseURL    string
	HTTPClient *http.Client
	Workers    int
}

func NewFetcher(baseURL string, workers int) *Fetcher {
	if workers <= 0 {
		workers = 8
	}
	return &Fetcher{
		BaseURL: baseURL,
		HTTPClient: &http.Client{
			Timeout: 120 * time.Second,
		},
		Workers: workers,
	}
}

func (f *Fetcher) FetchMatrix(nodes []*model.Node) (*model.Matrix, error) {
	n := len(nodes)
	if n == 0 {
		return nil, fmt.Errorf("no nodes provided")
	}

	if f.BaseURL != "" {
		return f.fetchFromOSRM(nodes)
	}
	return f.haversineFallback(nodes)
}

func (f *Fetcher) fetchFromOSRM(nodes []*model.Node) (*model.Matrix, error) {
	n := len(nodes)
	coordStr := ""
	for i, nd := range nodes {
		if i > 0 {
			coordStr += ";"
		}
		coordStr += fmt.Sprintf("%.6f,%.6f", nd.Lon, nd.Lat)
	}

	urlDist := fmt.Sprintf("%s/table/v1/driving/%s?annotations=distance", f.BaseURL, coordStr)
	urlTime := fmt.Sprintf("%s/table/v1/driving/%s?annotations=duration", f.BaseURL, coordStr)

	var (
		distMtx, timeMtx [][]float64
		distErr, timeErr error
		wg                sync.WaitGroup
	)

	wg.Add(2)
	go func() {
		defer wg.Done()
		distMtx, distErr = f.fetchTable(urlDist, "distances", n)
	}()
	go func() {
		defer wg.Done()
		timeMtx, timeErr = f.fetchTable(urlTime, "durations", n)
	}()
	wg.Wait()

	if distErr != nil {
		return nil, fmt.Errorf("distance fetch failed: %w", distErr)
	}
	if timeErr != nil {
		return nil, fmt.Errorf("duration fetch failed: %w", timeErr)
	}

	return &model.Matrix{Dist: distMtx, Time: timeMtx, N: n}, nil
}

func (f *Fetcher) fetchTable(url, annotation string, n int) ([][]float64, error) {
	resp, err := f.HTTPClient.Get(url)
	if err != nil {
		return nil, err
	}
	defer resp.Body.Close()

	body, err := io.ReadAll(resp.Body)
	if err != nil {
		return nil, err
	}

	var result osrmResponse
	if err := json.Unmarshal(body, &result); err != nil {
		return nil, err
	}
	if result.Code != "Ok" {
		return nil, fmt.Errorf("OSRM returned code: %s", result.Code)
	}

	mtx := result.Distances
	if annotation == "durations" {
		mtx = result.Durations
	}

	if len(mtx) != n {
		return nil, fmt.Errorf("OSRM matrix size mismatch: got %d, want %d", len(mtx), n)
	}

	for i := range mtx {
		for j := range mtx[i] {
			if i == j {
				mtx[i][j] = 0
			}
			if mtx[i][j] < 0 {
				mtx[i][j] = math.MaxFloat64 / 2
			}
		}
	}

	return mtx, nil
}

func (f *Fetcher) haversineFallback(nodes []*model.Node) (*model.Matrix, error) {
	n := len(nodes)
	dist := make([][]float64, n)
	tm := make([][]float64, n)
	for i := 0; i < n; i++ {
		dist[i] = make([]float64, n)
		tm[i] = make([]float64, n)
		for j := 0; j < n; j++ {
			d := haversine(nodes[i].Lat, nodes[i].Lon, nodes[j].Lat, nodes[j].Lon)
			dist[i][j] = d
			tm[i][j] = d / 500.0
		}
	}
	return &model.Matrix{Dist: dist, Time: tm, N: n}, nil
}

func haversine(lat1, lon1, lat2, lon2 float64) float64 {
	const R = 6371000.0
	dLat := (lat2 - lat1) * math.Pi / 180
	dLon := (lon2 - lon1) * math.Pi / 180
	a := math.Sin(dLat/2)*math.Sin(dLat/2) +
		math.Cos(lat1*math.Pi/180)*math.Cos(lat2*math.Pi/180)*
			math.Sin(dLon/2)*math.Sin(dLon/2)
	c := 2 * math.Atan2(math.Sqrt(a), math.Sqrt(1-a))
	return R * c * 1.3
}
