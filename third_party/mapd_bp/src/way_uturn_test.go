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
	matchNode := nodes.At(1)
	inAxisPoint := uTurnInAxisPoint(offline, current, true)

	ways, _ := offline.Ways()
	uTurn := ways.At(1)
	straight := wayByName(t, offline, "28th Avenue")

	if !isUTurn(uTurn, matchNode, inAxisPoint) {
		t.Error("expected reversal onto opposite carriageway to be flagged as U-turn")
	}
	if isUTurn(straight, matchNode, inAxisPoint) {
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

// A 90° side street must not be rejected: with road-axis bearings measured
// over U_TURN_AXIS_DIST meters on both sides of the junction, an ordinary
// same-name corner still measures ~90°, well inside the 150° threshold.
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
	inAxisPoint := uTurnInAxisPoint(offline, current, true)
	if isUTurn(cross, nodes.At(1), inAxisPoint) {
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

// Regression: real divided-carriageway crossover-jog geometry from the
// Nashville tile (31st Ave N S-curve, SOUTH merge node 36.147557,-86.816275).
// The SB carriageway's final ~9 m jog segment and the NB carriageway's first
// ~9 m jog segment rotate toward each other, so single adjacent-segment
// bearings measure the 180° carriageway reversal as only ~117° (cos -0.449)
// and used to pass the cos < -0.866 U-turn threshold. Road-axis bearings over
// >= 30 m measure ~170° and reject it, so NextWay must pick the straight
// two-way stub instead of the NB carriageway.
func crossoverJogFixture(t *testing.T) Offline {
	return buildOffline(t, []testWay{
		{
			// Tail of the SB carriageway (real nodes 15..18 of tile way idx 8888).
			name:   "31st Avenue North",
			oneWay: true,
			nodes: [][2]float64{
				{36.1481374, -86.8162968},
				{36.1480281, -86.8163025},
				{36.1476242, -86.8163251},
				{36.1475565, -86.8162752}, // south merge node
			},
		},
		{
			// NB carriageway (tile way idx 7883): doubles back north.
			name:   "31st Avenue North",
			oneWay: true,
			nodes: [][2]float64{
				{36.1475565, -86.8162752},
				{36.1476256, -86.8162205},
				{36.1480295, -86.8161978},
				{36.1481306, -86.8161893},
			},
		},
		{
			// Straight two-way stub (tile way idx 7884): continues south.
			name: "31st Avenue North",
			nodes: [][2]float64{
				{36.1475200, -86.8162777},
				{36.1475565, -86.8162752},
			},
		},
	})
}

func TestIsUTurnCrossoverJogGeometry(t *testing.T) {
	offline := crossoverJogFixture(t)
	ways, _ := offline.Ways()
	current := ways.At(0)
	nbCarriageway := ways.At(1)
	stub := ways.At(2)

	nodes, err := current.Nodes()
	if err != nil {
		t.Fatalf("could not read nodes: %v", err)
	}
	matchNode := nodes.At(nodes.Len() - 1)
	inAxisPoint := uTurnInAxisPoint(offline, current, true)

	if !isUTurn(nbCarriageway, matchNode, inAxisPoint) {
		t.Error("expected crossover jog onto the opposite carriageway to be flagged as U-turn")
	}
	if isUTurn(stub, matchNode, inAxisPoint) {
		t.Error("expected the straight two-way stub to not be flagged as U-turn")
	}
}

func TestNextWayPicksStubOverOppositeCarriagewayAtCrossover(t *testing.T) {
	offline := crossoverJogFixture(t)
	ways, _ := offline.Ways()
	current := ways.At(0)
	stub := ways.At(2)

	next, err := NextWay(current, offline, true)
	if err != nil {
		t.Fatalf("NextWay failed: %v", err)
	}
	if !next.Way.HasNodes() {
		t.Fatal("expected a next way")
	}
	if !isSameWay(next.Way, stub) {
		name, _ := next.Way.Name()
		nn, _ := next.Way.Nodes()
		t.Errorf("expected the straight stub, got %q first node (%f,%f)", name, nn.At(0).Latitude(), nn.At(0).Longitude())
	}
}

// Regression: crest crossover (tile node 36.1498956,-86.8179446). The NB
// carriageway ends in a 10.8 m stub way (tile idx 9023), shorter than
// U_TURN_AXIS_DIST, so the approach-axis walk must continue into the unique
// connected same-name way behind it (tile idx 6592) to establish the true
// road axis; only then does the reversal onto the SB carriageway (tile idx
// 8888) measure ~169° and get rejected, letting NextWay fall through the name
// tier to the renamed straight continuation "28th Avenue" (tile idx 1254).
func crestCrossoverFixture(t *testing.T) Offline {
	return buildOffline(t, []testWay{
		{
			// Tail of the NB carriageway approach (real nodes 7..9 of idx 6592).
			name:   "31st Avenue North",
			oneWay: true,
			lanes:  2,
			nodes: [][2]float64{
				{36.1492951, -86.8170433},
				{36.1494523, -86.8173345},
				{36.1498665, -86.8178301},
			},
		},
		{
			// 10.8 m crest stub (tile idx 9023).
			name:   "31st Avenue North",
			oneWay: true,
			lanes:  3,
			nodes: [][2]float64{
				{36.1498665, -86.8178301},
				{36.1498956, -86.8179446}, // crest node
			},
		},
		{
			// SB carriageway heading back down (first nodes of tile idx 8888).
			name:   "31st Avenue North",
			oneWay: true,
			nodes: [][2]float64{
				{36.1498956, -86.8179446},
				{36.1497990, -86.8179023},
				{36.1496974, -86.8177921},
				{36.1495874, -86.8176528},
			},
		},
		{
			// Renamed straight continuation over the crest (tile idx 1254).
			name:  "28th Avenue",
			lanes: 3,
			nodes: [][2]float64{
				{36.1498956, -86.8179446},
				{36.1499547, -86.8180126},
				{36.1504009, -86.8185268},
			},
		},
	})
}

func TestNextWayCrestFallsThroughToRenamedContinuation(t *testing.T) {
	offline := crestCrossoverFixture(t)
	ways, _ := offline.Ways()
	crestStub := ways.At(1)

	next, err := NextWay(crestStub, offline, true)
	if err != nil {
		t.Fatalf("NextWay failed: %v", err)
	}
	if !next.Way.HasNodes() {
		t.Fatal("expected a next way")
	}
	name, _ := next.Way.Name()
	if name != "28th Avenue" {
		t.Errorf("expected fall-through to the renamed straight continuation, got %q", name)
	}
}
