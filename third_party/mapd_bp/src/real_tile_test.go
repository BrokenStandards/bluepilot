package main

// Empirical checks against a real generated tile (Nashville,
// 36.00,-87.00..36.25,-86.75). These tests are skipped unless the
// MAPD_BP_REAL_TILE environment variable points at the tile file; they verify
// the crossover-junction fixes against the exact geometry that produced the
// defects (tile way indices: 8888 = SB carriageway of the 31st Ave N S-curve,
// 7883 = NB carriageway, 7884 = straight two-way stub at the south merge
// node, 9023 = 10.8 m NB crest stub, 6592 = NB approach, 1254 = renamed
// "28th Avenue" continuation over the crest).

import (
	"os"
	"testing"
)

const (
	realTileSBCarriageway = 8888
	realTileNBCarriageway = 7883
	realTileSouthStub     = 7884
	realTileCrestStub     = 9023
	realTileCrestRenamed  = 1254
)

func loadRealTile(t *testing.T) Offline {
	t.Helper()
	path := os.Getenv("MAPD_BP_REAL_TILE")
	if path == "" {
		t.Skip("MAPD_BP_REAL_TILE not set; skipping real-tile empirical tests")
	}
	data, err := os.ReadFile(path)
	if err != nil {
		t.Fatalf("could not read tile: %v", err)
	}
	offline := readOffline(data)
	ways, err := offline.Ways()
	if err != nil || ways.Len() == 0 {
		t.Fatalf("tile contains no ways (err=%v)", err)
	}
	return offline
}

// F1 (a): heading south on the SB carriageway, the next way at the south
// merge node must be the straight two-way stub, not the NB carriageway.
func TestRealTileSouthMergePicksStub(t *testing.T) {
	offline := loadRealTile(t)
	ways, _ := offline.Ways()

	next, err := NextWay(ways.At(realTileSBCarriageway), offline, true)
	if err != nil {
		t.Fatalf("NextWay failed: %v", err)
	}
	if !next.Way.HasNodes() {
		t.Fatal("expected a next way at the south merge node")
	}
	if isSameWay(next.Way, ways.At(realTileNBCarriageway)) {
		t.Fatal("NextWay chained onto the NB (opposite) carriageway: U-turn not rejected")
	}
	if !isSameWay(next.Way, ways.At(realTileSouthStub)) {
		name, _ := next.Way.Name()
		nn, _ := next.Way.Nodes()
		t.Fatalf("expected stub idx %d, got %q first node (%.7f,%.7f)",
			realTileSouthStub, name, nn.At(0).Latitude(), nn.At(0).Longitude())
	}
}

// F1 (b): heading north over the crest, the SB carriageway must be rejected in
// the name tier and NextWay must fall through to the renamed straight
// continuation "28th Avenue".
func TestRealTileCrestFallsThroughToRenamed(t *testing.T) {
	offline := loadRealTile(t)
	ways, _ := offline.Ways()

	next, err := NextWay(ways.At(realTileCrestStub), offline, true)
	if err != nil {
		t.Fatalf("NextWay failed: %v", err)
	}
	if !next.Way.HasNodes() {
		t.Fatal("expected a next way at the crest")
	}
	if isSameWay(next.Way, ways.At(realTileSBCarriageway)) {
		t.Fatal("NextWay chained onto the SB (opposite) carriageway at the crest")
	}
	name, _ := next.Way.Name()
	if name != "28th Avenue" || !isSameWay(next.Way, ways.At(realTileCrestRenamed)) {
		t.Fatalf("expected 28th Avenue (idx %d), got %q", realTileCrestRenamed, name)
	}
}

// F2: the read-traversal budget must survive many full-tile MatchingWays
// scans (each one re-reads every way's node list, ~3.6 MiB of traversal on
// this 11k-way tile) with node reads still working afterwards. Before the fix
// the default 64 MiB per-Message budget was exhausted after ~18 scans and all
// later reads silently returned empty structs.
func TestRealTileReadLimitSurvivesRepeatedScans(t *testing.T) {
	offline := loadRealTile(t)
	ways, _ := offline.Ways()
	way := ways.At(realTileSBCarriageway)
	nodes, err := way.Nodes()
	if err != nil || nodes.Len() < 2 {
		t.Fatalf("could not read seed way nodes (err=%v)", err)
	}
	matchNode := nodes.At(nodes.Len() - 1)

	const scans = 25 // > 18, the pre-fix budget exhaustion point
	for i := 0; i < scans; i++ {
		matching, err := MatchingWays(way, offline, matchNode)
		if err != nil {
			t.Fatalf("MatchingWays scan %d failed: %v", i, err)
		}
		if len(matching) == 0 {
			t.Fatalf("MatchingWays scan %d returned no ways: read budget exhausted", i)
		}
	}

	// Node reads must still return real data after all scans.
	nodes, err = way.Nodes()
	if err != nil {
		t.Fatalf("node read after scans failed: %v", err)
	}
	if nodes.Len() != 19 {
		t.Fatalf("expected 19 nodes on way %d after scans, got %d", realTileSBCarriageway, nodes.Len())
	}
	first := nodes.At(0)
	if first.Latitude() == 0 || first.Longitude() == 0 {
		t.Fatalf("node read returned zeroed coordinates after scans: read budget exhausted")
	}
	if name := RoadName(way); name != "31st Avenue North" {
		t.Fatalf("RoadName blanked after scans, got %q", name)
	}
}

// F3: on the southbound chain (28th Avenue -> SB carriageway) the published
// targets around the crest must keep the ~14.2 m/s real crest curve visible
// while the median-crossover jog artifact at the crossover node is suppressed
// or raised to >= ~12 m/s.
func TestRealTileCrestCurvatureTargets(t *testing.T) {
	offline := loadRealTile(t)
	ways, _ := offline.Ways()

	current := CurrentWay{
		Way:   ways.At(realTileCrestRenamed),
		OnWay: OnWayResult{OnWay: true, IsForward: false}, // southbound
	}
	nodes, _ := current.Way.Nodes()
	pos := Position{
		Latitude:  nodes.At(nodes.Len() - 1).Latitude(),
		Longitude: nodes.At(nodes.Len() - 1).Longitude(),
	}
	nextWays, err := NextWays(pos, current, offline, false)
	if err != nil {
		t.Fatalf("NextWays failed: %v", err)
	}
	if len(nextWays) == 0 || !isSameWay(nextWays[0].Way, ways.At(realTileSBCarriageway)) {
		t.Fatalf("expected the SB carriageway as the first next way")
	}

	state := &State{CurrentWay: current, NextWays: nextWays}
	curvatures, err := GetStateCurvatures(state)
	if err != nil {
		t.Fatalf("GetStateCurvatures failed: %v", err)
	}
	velocities := GetTargetVelocities(curvatures)

	velocityNear := func(lat, lon float64) float64 {
		best := -1.0
		bestDist := 5.0 // meters; must land on the exact node
		for _, v := range velocities {
			d := DistanceToPoint(lat*TO_RADIANS, lon*TO_RADIANS, v.Latitude*TO_RADIANS, v.Longitude*TO_RADIANS)
			if d < bestDist {
				bestDist = d
				best = v.Velocity
			}
		}
		return best
	}

	artifact := velocityNear(36.1497990, -86.8179023) // crossover node
	crest := velocityNear(36.1496974, -86.8177921)    // real crest curve
	t.Logf("southbound targets: crossover node = %.2f m/s, crest curve node = %.2f m/s", artifact, crest)

	if artifact > 0 && artifact < 12.0 {
		t.Errorf("crossover-node artifact still below 12 m/s: %.2f", artifact)
	}
	if crest <= 0 {
		t.Fatal("no target published at the crest curve node")
	}
	if crest < 13.0 || crest > 15.5 {
		t.Errorf("real crest curve target no longer ~14.2 m/s: %.2f", crest)
	}

	// The approach minimum around the crest (which may still sit at the
	// crossover node) must satisfy the acceptance floor of >= ~12 m/s — before
	// the fix it was the 9.59 m/s artifact.
	min := -1.0
	for _, v := range velocities {
		d := DistanceToPoint(36.1496974*TO_RADIANS, -86.8177921*TO_RADIANS, v.Latitude*TO_RADIANS, v.Longitude*TO_RADIANS)
		if d <= 40 && v.Velocity > 0 && (min < 0 || v.Velocity < min) {
			min = v.Velocity
		}
	}
	t.Logf("southbound approach minimum within 40 m of crest curve: %.2f m/s", min)
	if min < 12.0 {
		t.Errorf("approach minimum around the crest is %.2f m/s; artifact not suppressed", min)
	}
}
