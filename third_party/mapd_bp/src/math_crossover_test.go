package main

import (
	"testing"
)

// Real median-crossover geometry from the Nashville tile: southbound over the
// 31st Ave N crest, the two-way "28th Avenue" (tile idx 1254, traversed
// backward) hands over to the oneway SB carriageway (tile idx 8888) at the
// crest node (36.1498956,-86.8179446). The boundary-adjacent segments (9.0 m
// and 11.4 m) are the median-crossover jog; the node triples centered on the
// boundary fake a hard bend (~0.022 1/m, a 9.6 m/s target) that no traffic
// drives, while the REAL crest curve (~0.010 1/m, ~14.2 m/s) peaks ~14 m
// further on and must survive.
func crossoverChainState(t *testing.T, nextOneWay bool) *State {
	offline := buildOffline(t, []testWay{
		{
			name:  "28th Avenue",
			lanes: 3,
			nodes: [][2]float64{
				{36.1498956, -86.8179446}, // crest node (boundary)
				{36.1499547, -86.8180126},
				{36.1504009, -86.8185268},
			},
		},
		{
			name:   "31st Avenue North",
			oneWay: nextOneWay,
			nodes: [][2]float64{
				{36.1498956, -86.8179446},
				{36.1497990, -86.8179023}, // crossover node (jog artifact)
				{36.1496974, -86.8177921}, // real crest curve peak
				{36.1495874, -86.8176528},
				{36.1494768, -86.8175158},
				{36.1493492, -86.8173391},
				{36.1492361, -86.8171492},
				{36.1491406, -86.8169873},
				{36.1490614, -86.8168426},
				{36.1489552, -86.8166853},
			},
		},
	})
	ways, err := offline.Ways()
	if err != nil {
		t.Fatalf("could not read ways: %v", err)
	}
	return &State{
		CurrentWay: CurrentWay{
			Way:   ways.At(0),
			OnWay: OnWayResult{OnWay: true, IsForward: false}, // southbound
		},
		NextWays: []NextWayResult{
			{Way: ways.At(1), IsForward: true},
		},
	}
}

func curvatureNear(t *testing.T, state *State, lat, lon float64) float64 {
	t.Helper()
	curvatures, err := GetStateCurvatures(state)
	if err != nil {
		t.Fatalf("GetStateCurvatures failed: %v", err)
	}
	for _, c := range curvatures {
		d := DistanceToPoint(lat*TO_RADIANS, lon*TO_RADIANS, c.Latitude*TO_RADIANS, c.Longitude*TO_RADIANS)
		if d < 3 {
			return c.Curvature
		}
	}
	t.Fatalf("no curvature output at (%f,%f)", lat, lon)
	return 0
}

// The two-way<->oneway boundary with short (< 15 m) adjacent segments is a
// median crossover: the jog artifact at the crossover node must be suppressed
// (target >= ~12 m/s means curvature <= ~0.014 1/m) while the real crest
// curve one node further keeps its ~0.010 1/m (~14.2 m/s) target.
func TestCrossoverJogArtifactSuppressed(t *testing.T) {
	state := crossoverChainState(t, true)

	artifact := curvatureNear(t, state, 36.1497990, -86.8179023)
	crest := curvatureNear(t, state, 36.1496974, -86.8177921)

	if artifact > 0.014 {
		t.Errorf("crossover-node artifact not suppressed: curvature %f (> 0.014, i.e. target < 12 m/s)", artifact)
	}
	if crest < 0.008 || crest > 0.012 {
		t.Errorf("real crest curve no longer ~0.010 1/m: %f", crest)
	}
}

// Control: identical geometry WITHOUT the two-way<->oneway transition is not
// classified as a crossover, so the (then plausibly real) boundary curvature
// is left untouched.
func TestCrossoverSuppressionRequiresOneWayTransition(t *testing.T) {
	state := crossoverChainState(t, false)

	artifact := curvatureNear(t, state, 36.1497990, -86.8179023)
	if artifact < 0.02 {
		t.Errorf("expected raw boundary curvature (~0.022) without a oneway transition, got %f", artifact)
	}
}
