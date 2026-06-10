package model

type TimeWindow struct {
	Earliest float64 `json:"earliest"`
	Latest   float64 `json:"latest"`
}

type Node struct {
	ID      int       `json:"id"`
	Lat     float64   `json:"lat"`
	Lon     float64   `json:"lon"`
	Demand  float64   `json:"demand"`
	Volume  float64   `json:"volume"`
	TW      TimeWindow `json:"tw"`
	Service float64   `json:"service"`
	IsDepot bool      `json:"is_depot"`
	DepotID int       `json:"depot_id"`
}

type Vehicle struct {
	ID        int     `json:"id"`
	DepotID   int     `json:"depot_id"`
	CapWeight float64 `json:"cap_weight"`
	CapVolume float64 `json:"cap_volume"`
	Count     int     `json:"count"`
}

type Matrix struct {
	Dist [][]float64
	Time [][]float64
	N    int
}

type Route struct {
	Vehicle     *Vehicle
	Nodes       []int
	LoadW       float64
	LoadV       float64
	Dist        float64
	Time        float64
	DepartDepot float64
}

type Solution struct {
	Routes    []*Route
	TotalDist float64
	TotalTime float64
	Feasible  bool
	Unserved  []int
}

type Problem struct {
	Nodes     []*Node
	Vehicles  []*Vehicle
	Matrix    *Matrix
	Depots    []*Node
	Customers []*Node
}

type SolveRequest struct {
	Nodes    []*Node    `json:"nodes"`
	Vehicles []*Vehicle `json:"vehicles"`
	OSRMURL  string     `json:"osrm_url,omitempty"`
}

type RouteResult struct {
	VehicleID  int     `json:"vehicle_id"`
	DepotID    int     `json:"depot_id"`
	Sequence   []int   `json:"sequence"`
	Distance   float64 `json:"distance"`
	Duration   float64 `json:"duration"`
	LoadWeight float64 `json:"load_weight"`
	LoadVolume float64 `json:"load_volume"`
}

type SolveResponse struct {
	Routes     []RouteResult `json:"routes"`
	TotalDist  float64       `json:"total_distance"`
	TotalTime  float64       `json:"total_duration"`
	Feasible   bool          `json:"feasible"`
	Unserved   []int         `json:"unserved,omitempty"`
	Iterations int           `json:"iterations"`
}
