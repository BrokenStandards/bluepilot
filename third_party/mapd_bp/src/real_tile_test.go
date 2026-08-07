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
	"math"
	"os"
	"testing"
)

const (
	realTileSBCarriageway = 8888
	realTileNBCarriageway = 7883
	realTileSouthStub     = 7884
	realTileCrestStub     = 9023
	realTileCrestRenamed  = 1254
	// Two-way 31st Ave N south of the pre-light bend; traversed forward this
	// is the northbound approach, traversed backward the southbound one. It
	// starts two nodes before the pre-light bend, so the bend node gets its
	// own published sample (output i is anchored at x_points[i+2]).
	realTilePreLightApproach = 10182
	realTilePreLightFromNB   = 10180
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

// chainVelocities replays a chain starting at one end of the given tile way
// and returns the published curve targets, plus a lookup by position.
func chainVelocities(t *testing.T, offline Offline, wayIdx int, forward bool) ([]Velocity, func(lat, lon float64) float64) {
	t.Helper()
	ways, _ := offline.Ways()
	way := ways.At(wayIdx)
	nodes, err := way.Nodes()
	if err != nil || nodes.Len() < 2 {
		t.Fatalf("could not read way %d nodes: %v", wayIdx, err)
	}
	start := nodes.At(0)
	if !forward {
		start = nodes.At(nodes.Len() - 1)
	}
	current := CurrentWay{Way: way, OnWay: OnWayResult{OnWay: true, IsForward: forward}}
	pos := Position{Latitude: start.Latitude(), Longitude: start.Longitude()}
	nextWays, err := NextWays(pos, current, offline, forward)
	if err != nil {
		t.Fatalf("NextWays failed on way %d: %v", wayIdx, err)
	}
	curvatures, err := GetStateCurvatures(&State{CurrentWay: current, NextWays: nextWays})
	if err != nil {
		t.Fatalf("GetStateCurvatures failed on way %d: %v", wayIdx, err)
	}
	velocities := GetTargetVelocities(curvatures)
	at := func(lat, lon float64) float64 {
		best, bestDist := -1.0, 5.0 // meters; must land on the exact node
		for _, v := range velocities {
			d := DistanceToPoint(lat*TO_RADIANS, lon*TO_RADIANS, v.Latitude*TO_RADIANS, v.Longitude*TO_RADIANS)
			if d < bestDist {
				bestDist, best = d, v.Velocity
			}
		}
		return best
	}
	return velocities, at
}

// G1 acceptance: the 31st Ave N pre-light bend. The raw circumcircle triple
// centred on the bend node gives R = 66.2 m; before peak preservation the
// 3-sample average diluted it to R = 166.0 m (18.22 m/s / 40.8 mph) for a
// bend the driver takes at 54-67 m radius and brakes manually for in BOTH
// directions. The published radius must now be the raw peak.
func TestRealTilePreLightBendKeepsItsPeak(t *testing.T) {
	offline := loadRealTile(t)

	for _, tc := range []struct {
		name    string
		wayIdx  int
		forward bool
	}{
		{"northbound", realTilePreLightApproach, true},
		{"southbound", realTilePreLightFromNB, false},
	} {
		_, at := chainVelocities(t, offline, tc.wayIdx, tc.forward)
		v := at(36.1440162, -86.8165422)
		if v <= 0 {
			t.Fatalf("%s: no target published at the pre-light bend", tc.name)
		}
		radius := v * v / TARGET_LAT_ACCEL
		t.Logf("%s pre-light bend: %.2f m/s (R = %.1f m)", tc.name, v, radius)
		if radius > 80 {
			t.Errorf("%s: pre-light bend still smeared: R = %.1f m (%.2f m/s), expected near the raw 66.2 m",
				tc.name, radius, v)
		}
		if radius < 55 {
			t.Errorf("%s: pre-light bend sharper than the raw peak: R = %.1f m", tc.name, radius)
		}
	}
}

// G2 acceptance: the median crossover at the south merge node. The chain
// 31st Ave N (two-way) -> 31st Ave N (oneway NB carriageway) jogs across the
// median there; the flattened input triples still let an unflattened
// neighbour dominate the 3-window average and published 0.013292 1/m, the
// 12.267 m/s phantom logged 79 times on this route. Nothing on the jog may
// publish below ~20 m/s now, and the real bends further along the same chain
// must keep their targets.
func TestRealTileMergeCrossoverPhantomGone(t *testing.T) {
	offline := loadRealTile(t)
	velocities, at := chainVelocities(t, offline, realTilePreLightApproach, true)

	const crossLat, crossLon = 36.1475565, -86.8162752
	v := at(crossLat, crossLon)
	t.Logf("northbound merge crossover target: %.2f m/s", v)
	if v > 0 && v < 20.0 {
		t.Errorf("merge-crossover phantom still present: %.2f m/s at (%.7f,%.7f)", v, crossLat, crossLon)
	}

	// Nothing within the jog itself (the two short boundary-adjacent
	// segments, both under CROSSOVER_MAX_BOUNDARY_SEGMENT) may bind either.
	for _, p := range velocities {
		d := DistanceToPoint(crossLat*TO_RADIANS, crossLon*TO_RADIANS, p.Latitude*TO_RADIANS, p.Longitude*TO_RADIANS)
		if d <= CROSSOVER_MAX_BOUNDARY_SEGMENT && p.Velocity > 0 && p.Velocity < 20.0 {
			t.Errorf("jog sample %.1f m from the crossover still publishes %.2f m/s at (%.7f,%.7f)",
				d, p.Velocity, p.Latitude, p.Longitude)
		}
	}

	// Real bends on the same chain keep their targets: the pre-light bend
	// sharpens to its raw peak and the S-curve entry north of the merge is
	// unchanged.
	if pre := at(36.1440162, -86.8165422); pre <= 0 || pre > 13.0 {
		t.Errorf("pre-light bend target lost on this chain: %.2f m/s", pre)
	}
	if s := at(36.1480295, -86.8161978); s < 14.0 || s > 16.5 {
		t.Errorf("S-curve entry north of the merge moved: %.2f m/s (expected ~15.1)", s)
	}
}

// The northbound bridge inner curve is the one place on the route where the
// map radius (86.1 m) matches the driven radius, MTSC already fired and the
// car pulled the 2.0 m/s^2 design point. Peak preservation must leave it
// alone — if the max moves this one, it is firing where the average was
// already right.
func TestRealTileBridgeInnerCurveUnchanged(t *testing.T) {
	offline := loadRealTile(t)
	_, at := chainVelocities(t, offline, realTileNBCarriageway, true)

	v := at(36.1511278, -86.8191728)
	if v <= 0 {
		t.Fatal("no target published at the bridge inner curve")
	}
	radius := v * v / TARGET_LAT_ACCEL
	t.Logf("northbound bridge inner curve: %.2f m/s (R = %.1f m)", v, radius)
	if math.Abs(radius-86.1) > 5.0 {
		t.Errorf("well-modelled bridge inner curve moved: R = %.1f m (was 86.1 m)", radius)
	}
}
