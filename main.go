package main

import (
	"flag"
	"fmt"
	"os"

	"github.com/logistics/vrptw-engine/api"
)

func main() {
	port := flag.String("port", ":8080", "HTTP listen address")
	osrmURL := flag.String("osrm", "", "OSRM server URL (e.g. http://localhost:5000)")
	flag.Parse()

	if envPort := os.Getenv("VRPTW_PORT"); envPort != "" {
		*port = envPort
	}
	if envOSRM := os.Getenv("OSRM_URL"); envOSRM != "" {
		*osrmURL = envOSRM
	}

	fmt.Printf("=== VRPTW Cold Chain Scheduling Engine ===\n")
	fmt.Printf("Port: %s | OSRM: %s\n", *port, func() string {
		if *osrmURL == "" {
			return "haversine fallback"
		}
		return *osrmURL
	}())

	server := api.NewServer(*port, *osrmURL)
	if err := server.Start(); err != nil {
		fmt.Fprintf(os.Stderr, "Server error: %v\n", err)
		os.Exit(1)
	}
}
