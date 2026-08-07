package main

import (
	"math"
	"testing"
)

// Peak-preserving curvature (G1) and post-average boundary suppression (G2).
//
// The 3-sample arc-length-weighted average in GetAverageCurvatures smears a
// curvature peak into its straight neighbours. GetStateCurvatures now
// publishes max(average, raw curvature of the triple centred on the same
// node), so the average can only soften the approach, never erase the peak —
// except where the merge/split or crossover passes deliberately flattened the
// sample, which must stay flattened.

// A sharp bend flanked by near-straight legs: the raw triple centred on the
// bend node has a small radius, the two neighbouring triples are nearly
// straight, and all three carry comparable arc-length weights, so the average
// dilutes the peak by roughly 3x. The published value must be the peak.
func peakState(t *testing.T) *State {
	t.Helper()
	// ~70 m spacing heading north, a 64-degree kink at the middle node
	// (R ~ 66 m for the centred triple), then ~70 m spacing heading
	// north-east. Tiny alternating longitude jitter keeps the "straight"
	// triples non-degenerate: exactly collinear points give a zero area,
	// an infinite radius and a NaN arc length.
	offline := buildOffline(t, []testWay{
		{
			name: "Peak Road",
			nodes: [][2]float64{
				{36.1387410, -86.8100010},
				{36.1393705, -86.8099990},
				{36.1400000, -86.8100010},
				{36.1406295, -86.8099990},
				{36.1412590, -86.8100010},
				{36.1418885, -86.8100000}, // sharp bend node
				{36.1421646, -86.8093011},
				{36.1424407, -86.8086012},
				{36.1427168, -86.8079033},
				{36.1429929, -86.8072034},
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
			OnWay: OnWayResult{OnWay: true, IsForward: true},
		},
	}
}

func publishedNear(t *testing.T, curvatures []Curvature, lat, lon float64) float64 {
	t.Helper()
	for _, c := range curvatures {
		if DistanceToPoint(lat*TO_RADIANS, lon*TO_RADIANS, c.Latitude*TO_RADIANS, c.Longitude*TO_RADIANS) < 3 {
			return c.Curvature
		}
	}
	t.Fatalf("no published curvature at (%f,%f)", lat, lon)
	return 0
}

func TestPeakSurvivesAveraging(t *testing.T) {
	state := peakState(t)
	nodes, err := state.CurrentWay.Way.Nodes()
	if err != nil {
		t.Fatalf("could not read nodes: %v", err)
	}
	xs := make([]float64, nodes.Len())
	ys := make([]float64, nodes.Len())
	for i := 0; i < nodes.Len(); i++ {
		xs[i] = nodes.At(i).Latitude()
		ys[i] = nodes.At(i).Longitude()
	}
	raw, arcs, err := GetCurvatures(xs, ys)
	if err != nil {
		t.Fatalf("GetCurvatures failed: %v", err)
	}
	avg, err := GetAverageCurvatures(raw, arcs)
	if err != nil {
		t.Fatalf("GetAverageCurvatures failed: %v", err)
	}

	// The bend node is xs[5]; the raw triple centred on it is raw[4] and the
	// output anchored on it is index 3 (output i is anchored at x_points[i+2]).
	const bendNode = 5
	rawPeak := raw[bendNode-1]
	averaged := avg[bendNode-2]
	rawRadius := 1.0 / rawPeak
	avgRadius := 1.0 / averaged
	t.Logf("bend node: raw R = %.1f m (%.2f m/s), averaged R = %.1f m (%.2f m/s)",
		rawRadius, math.Sqrt(TARGET_LAT_ACCEL*rawRadius), avgRadius, math.Sqrt(TARGET_LAT_ACCEL*avgRadius))

	// Sanity: the fixture really does exercise smearing.
	if rawRadius > 80 {
		t.Fatalf("fixture broken: expected a sharp (~66 m) raw peak, got R = %.1f m", rawRadius)
	}
	if avgRadius < rawRadius*2 {
		t.Fatalf("fixture broken: expected the average to dilute the peak >2x, raw R = %.1f m, avg R = %.1f m", rawRadius, avgRadius)
	}

	curvatures, err := GetStateCurvatures(state)
	if err != nil {
		t.Fatalf("GetStateCurvatures failed: %v", err)
	}
	got := publishedNear(t, curvatures, xs[bendNode], ys[bendNode])
	if math.Abs(got-rawPeak) > 1e-9 {
		t.Errorf("peak not preserved: published %f (R = %.1f m), raw peak %f (R = %.1f m)",
			got, 1.0/got, rawPeak, rawRadius)
	}

	// Straight sections must not gain curvature: the published value there is
	// still bounded by the sharper of the local average and the local raw
	// sample, both near zero.
	straight := publishedNear(t, curvatures, xs[3], ys[3])
	if straight > 0.001 {
		t.Errorf("straight section gained curvature: %f", straight)
	}
}

// Ordering: the crossover pass flattens the input triples BEFORE the average,
// and the boundary-anchored outputs are flattened AFTER the max, so the raw
// jog curvature can never be resurrected by the peak-preserving max. On this
// fixture the raw triple centred on the crossover node is ~0.0285 1/m (an
// 8.4 m/s phantom); the published value must stay at FLATTENED_CURVATURE.
func TestCrossoverSuppressionSurvivesPeakMax(t *testing.T) {
	suppressed := crossoverChainState(t, true)
	control := crossoverChainState(t, false)

	// Without the oneway transition nothing is suppressed, so the published
	// value at the crossover node is the raw jog peak — proof that the max
	// would resurrect it if the suppression ran in the wrong order.
	rawJog := curvatureNear(t, control, 36.1497990, -86.8179023)
	if rawJog < 0.02 {
		t.Fatalf("fixture broken: expected a raw jog peak >= 0.02 1/m, got %f", rawJog)
	}

	for _, p := range [][2]float64{
		{36.1498956, -86.8179446}, // boundary node
		{36.1497990, -86.8179023}, // crossover node (the logged phantom)
	} {
		got := curvatureNear(t, suppressed, p[0], p[1])
		if got != FLATTENED_CURVATURE {
			t.Errorf("boundary jog output at (%.7f,%.7f) not flattened: %f (raw jog peak is %f)",
				p[0], p[1], got, rawJog)
		}
	}

	// The real curve the crossover sits on is one node further along and must
	// still be published.
	crest := curvatureNear(t, suppressed, 36.1496974, -86.8177921)
	if crest < 0.008 || crest > 0.012 {
		t.Errorf("real crest curve no longer ~0.010 1/m: %f", crest)
	}
}
