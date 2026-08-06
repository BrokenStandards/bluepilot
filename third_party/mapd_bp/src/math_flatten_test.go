package main

import (
	"testing"
)

// State fixture: a straight current way followed by a next way that bends
// east right at the boundary, with configurable lane counts. The bend is a
// real curve target that merge/split flattening would erase.
func flattenState(t *testing.T, currentLanes uint8, nextLanes uint8) *State {
	offline := buildOffline(t, []testWay{
		{
			name:  "Crest Road",
			lanes: currentLanes,
			nodes: [][2]float64{
				{40.000, -83.000001},
				{40.001, -83.000002},
				{40.002, -83.000001},
				{40.003, -83.000},
			},
		},
		{
			name:  "Crest Road",
			lanes: nextLanes,
			nodes: [][2]float64{
				{40.003, -83.000},
				{40.0035, -82.9995},
				{40.0037, -82.9985},
				{40.0037, -82.997},
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
		NextWays: []NextWayResult{
			{Way: ways.At(1), IsForward: true},
		},
	}
}

func maxCurvature(t *testing.T, state *State) float64 {
	t.Helper()
	curvatures, err := GetStateCurvatures(state)
	if err != nil {
		t.Fatalf("GetStateCurvatures failed: %v", err)
	}
	maxCurv := 0.0
	for _, c := range curvatures {
		if c.Curvature > maxCurv {
			maxCurv = c.Curvature
		}
	}
	return maxCurv
}

// Unknown (untagged, 0) lane count on either side must NOT trigger merge/split
// flattening: 0 -> 2 "looks like" a lane increase but is just missing data,
// and flattening it erased real curve targets at tagged/untagged boundaries.
func TestFlatteningSkippedWhenLaneCountUnknown(t *testing.T) {
	unknownLanes := maxCurvature(t, flattenState(t, 0, 2))
	knownLanes := maxCurvature(t, flattenState(t, 2, 4))

	if unknownLanes <= knownLanes {
		t.Errorf("expected real curvature to survive with unknown lane counts: unknown=%f known=%f", unknownLanes, knownLanes)
	}
	if unknownLanes < 0.003 {
		t.Errorf("expected the real bend (~0.005 1/m) to be preserved, got %f", unknownLanes)
	}
}

// Control: with both lane counts tagged, a genuine lane-count increase still
// flattens the boundary curvature exactly as before the fork.
func TestFlatteningStillAppliesWhenBothLaneCountsKnown(t *testing.T) {
	flattenedMax := maxCurvature(t, flattenState(t, 2, 4))
	if flattenedMax >= 0.003 {
		t.Errorf("expected boundary curvature to be flattened with known lane counts, got max %f", flattenedMax)
	}
}
