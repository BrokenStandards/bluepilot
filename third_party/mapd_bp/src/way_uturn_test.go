package main

import (
	"testing"
)

// The current way heads due north and ends at (40.001, -83). The same-name
// candidate doubles back onto the opposite carriageway (a divided-road U-turn
// whose junction circumcircle is nearly flat, so isValidConnection passes it);
// the physically straight continuation was renamed. The name tier must fall
// through so the straight way wins in the any-valid tier.
func uTurnFixture(t *testing.T) Offline {
	return buildOffline(t, []testWay{
		{
			name:  "31st Avenue North",
			nodes: [][2]float64{{40.000, -83.000}, {40.001, -83.000}},
		},
		{
			// Opposite carriageway: starts at the merge node, heads back south
			// with ~8 m of lateral offset over ~110 m.
			name:   "31st Avenue North",
			oneWay: true,
			nodes:  [][2]float64{{40.001, -83.000}, {40.0000, -83.0001}},
		},
		{
			// Slight lon offset keeps the junction triple non-colinear so the
			// curvature math stays well defined.
			name:  "28th Avenue",
			nodes: [][2]float64{{40.001, -83.000}, {40.002, -82.99999}},
		},
	})
}

func TestIsUTurnGeometry(t *testing.T) {
	offline := uTurnFixture(t)
	current := wayByName(t, offline, "31st Avenue North")
	nodes, err := current.Nodes()
	if err != nil {
		t.Fatalf("could not read nodes: %v", err)
	}
	bearingNode := nodes.At(0)
	matchNode := nodes.At(1)

	ways, _ := offline.Ways()
	uTurn := ways.At(1)
	straight := wayByName(t, offline, "28th Avenue")

	if !isUTurn(uTurn, matchNode, bearingNode) {
		t.Error("expected reversal onto opposite carriageway to be flagged as U-turn")
	}
	if isUTurn(straight, matchNode, bearingNode) {
		t.Error("expected straight continuation to not be flagged as U-turn")
	}
}

func TestNextWayNameTierFallsThroughOnUTurn(t *testing.T) {
	offline := uTurnFixture(t)
	current := wayByName(t, offline, "31st Avenue North")

	next, err := NextWay(current, offline, true)
	if err != nil {
		t.Fatalf("NextWay failed: %v", err)
	}
	if !next.Way.HasNodes() {
		t.Fatal("expected a next way")
	}
	name, _ := next.Way.Name()
	if name != "28th Avenue" {
		t.Errorf("expected name tier to fall through to the straight renamed way, got %q", name)
	}
}

// A 90° side street must not be rejected: only direction reversals beyond 150°
// are U-turns. Ordinary intersections keep the pre-fork behavior.
func TestIsUTurnAllowsRightAngleTurn(t *testing.T) {
	offline := buildOffline(t, []testWay{
		{
			name:  "Main Street",
			nodes: [][2]float64{{40.000, -83.000}, {40.001, -83.000}},
		},
		{
			name:  "Cross Street",
			nodes: [][2]float64{{40.001, -83.000}, {40.001, -82.999}},
		},
	})
	current := wayByName(t, offline, "Main Street")
	nodes, _ := current.Nodes()
	cross := wayByName(t, offline, "Cross Street")
	if isUTurn(cross, nodes.At(1), nodes.At(0)) {
		t.Error("90 degree turn must not be classified as a U-turn")
	}
}

// When the ONLY connected way is a U-turn (here it also fails the oneway
// drivability check, so it reaches the last-resort fallback), the fallback
// must return no next way instead of chaining onto the reversal.
func TestNextWayFallbackRejectsUTurn(t *testing.T) {
	offline := buildOffline(t, []testWay{
		{
			// Unnamed current way so the name/ref tiers are skipped.
			nodes: [][2]float64{{40.000, -83.000}, {40.001, -83.000}},
		},
		{
			name:   "Opposite Carriageway",
			oneWay: true,
			// Ends at the match node; traversing away from it heads back south.
			nodes: [][2]float64{{40.0000, -83.0001}, {40.001, -83.000}},
		},
	})
	ways, _ := offline.Ways()
	current := ways.At(0)

	next, err := NextWay(current, offline, true)
	if err != nil {
		t.Fatalf("NextWay failed: %v", err)
	}
	if next.Way.HasNodes() {
		name, _ := next.Way.Name()
		t.Errorf("expected no next way from the fallback, got %q", name)
	}
}
