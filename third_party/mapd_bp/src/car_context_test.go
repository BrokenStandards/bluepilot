package main

import (
	"math"
	"testing"
)

// Nearly straight way (tiny jitter keeps the curvature math well defined).
func straightWayOffline(t *testing.T) Offline {
	return buildOffline(t, []testWay{
		{
			name: "Straight Street",
			nodes: [][2]float64{
				{39.9995, -83.000001},
				{40.0000, -83.000002},
				{40.0005, -83.000001},
				{40.0010, -83.000},
			},
		},
	})
}

func divergenceState(t *testing.T, enabled bool, carCurvature float64) *State {
	offline := straightWayOffline(t)
	ways, _ := offline.Ways()
	return &State{
		CurrentWay: CurrentWay{
			Way:   ways.At(0),
			OnWay: OnWayResult{OnWay: true, IsForward: true},
		},
		Position:   Position{Latitude: 40.0000, Longitude: -83.000},
		CarContext: CarContext{Enabled: enabled, Curvature: carCurvature},
	}
}

// Three consecutive diverged 1 Hz ticks trigger a rematch; the counter then
// resets so a failed rematch cannot re-run the full scan every tick.
func TestDivergenceTriggersAfterThreeTicks(t *testing.T) {
	state := divergenceState(t, true, 0.02) // way is straight, car is turning hard
	for tick := 1; tick <= 2; tick++ {
		if UpdateDivergence(state) {
			t.Fatalf("tick %d: rematch triggered too early", tick)
		}
	}
	if !UpdateDivergence(state) {
		t.Fatal("tick 3: expected rematch trigger")
	}
	if state.DivergenceTicks != 0 {
		t.Errorf("expected counter reset after trigger, got %d", state.DivergenceTicks)
	}
	if UpdateDivergence(state) {
		t.Error("tick 4: expected a fresh streak to be required after a trigger")
	}
}

func TestDivergenceDisabledWithoutCarContext(t *testing.T) {
	state := divergenceState(t, false, 0.02)
	for tick := 0; tick < 5; tick++ {
		if UpdateDivergence(state) {
			t.Fatal("divergence must never trigger when MapdCarContext is not enabled")
		}
	}
	if state.DivergenceTicks != 0 {
		t.Errorf("expected counter to stay reset while disabled, got %d", state.DivergenceTicks)
	}
}

func TestDivergenceResetsOnAgreement(t *testing.T) {
	state := divergenceState(t, true, 0.02)
	UpdateDivergence(state)
	UpdateDivergence(state)
	// Car straightens out: curvatures agree again (both ~0).
	state.CarContext.Curvature = 0.0
	if UpdateDivergence(state) {
		t.Fatal("agreement tick must not trigger")
	}
	if state.DivergenceTicks != 0 {
		t.Errorf("expected counter reset on agreement, got %d", state.DivergenceTicks)
	}
	// Small disagreements below both thresholds never count.
	state.CarContext.Curvature = 0.004
	if UpdateDivergence(state) || state.DivergenceTicks != 0 {
		t.Error("sub-threshold disagreement must not accumulate ticks")
	}
}

// Two near-parallel ways ~6 m apart: "Alpha Street" is straight, "Beta
// Street" is a radius-50 m arc whose apex tangent is due north. The car sits
// on Alpha but reports 0.02 1/m of curvature matching Beta.
func rematchOffline(t *testing.T) Offline {
	// Meter offsets converted to degrees around (40.0, -83.0).
	latDeg := func(m float64) float64 { return 40.0 + m/111320.0 }
	lonDeg := func(m float64) float64 { return -83.0 + m/(111320.0*0.766) }

	arc := func(theta float64) [2]float64 {
		return [2]float64{latDeg(50 * math.Sin(theta)), lonDeg(50 * (1 - math.Cos(theta)))}
	}
	return buildOffline(t, []testWay{
		{
			name: "Alpha Street",
			nodes: [][2]float64{
				{latDeg(-30), lonDeg(6.000001)},
				{latDeg(-10), lonDeg(6.000002)},
				{latDeg(10), lonDeg(6.000001)},
				{latDeg(30), lonDeg(6.0)},
			},
		},
		{
			name:  "Beta Street",
			nodes: [][2]float64{arc(-0.6), arc(-0.3), arc(0), arc(0.3), arc(0.6)},
		},
	})
}

func TestRematchSwitchesToCurvatureMatchingWay(t *testing.T) {
	offline := rematchOffline(t)
	alpha := wayByName(t, offline, "Alpha Street")
	current := CurrentWay{Way: alpha, OnWay: OnWayResult{OnWay: true, IsForward: true}}
	pos := Position{Latitude: 40.0, Longitude: -83.0 + 6.0/(111320.0*0.766), Bearing: 0}

	// Without a rematch the sticky branch keeps Alpha.
	sticky, err := GetCurrentWay(current, nil, offline, pos, pos, 5.0, false, 0.02)
	if err != nil {
		t.Fatalf("GetCurrentWay failed: %v", err)
	}
	if name, _ := sticky.Way.Name(); name != "Alpha Street" {
		t.Fatalf("expected sticky match to stay on Alpha Street, got %q", name)
	}

	// With a rematch, Beta's matching curvature outscores Alpha.
	rematched, err := GetCurrentWay(current, nil, offline, pos, pos, 5.0, true, 0.02)
	if err != nil {
		t.Fatalf("GetCurrentWay failed: %v", err)
	}
	if name, _ := rematched.Way.Name(); name != "Beta Street" {
		t.Errorf("expected rematch to switch to the curvature-matching way, got %q", name)
	}
}

// Fail-safe: when no candidate strictly outscores the current way (here it is
// the only way at all), a triggered rematch must keep the current match.
func TestRematchFailsSafeWithoutBetterCandidate(t *testing.T) {
	offline := straightWayOffline(t)
	ways, _ := offline.Ways()
	current := CurrentWay{Way: ways.At(0), OnWay: OnWayResult{OnWay: true, IsForward: true}}
	pos := Position{Latitude: 40.0, Longitude: -83.000001, Bearing: 0}

	result, err := GetCurrentWay(current, nil, offline, pos, pos, 5.0, true, 0.02)
	if err != nil {
		t.Fatalf("GetCurrentWay failed: %v", err)
	}
	if name, _ := result.Way.Name(); name != "Straight Street" {
		t.Errorf("expected fail-safe to keep the current way, got %q", name)
	}
}
