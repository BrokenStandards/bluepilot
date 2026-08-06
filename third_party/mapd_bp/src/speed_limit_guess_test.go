package main

import (
	"math"
	"testing"
)

const (
	mph35 = 35 * 0.44704
	mph40 = 40 * 0.44704
)

// Chain of three same-name ways heading north; the middle one is the current
// way and has no maxspeed tag.
func guessOffline(t *testing.T, backSpeed float64, forwardSpeed float64) Offline {
	return buildOffline(t, []testWay{
		{
			name:     "Charlotte Avenue",
			maxSpeed: backSpeed,
			nodes:    [][2]float64{{40.000, -83.000}, {40.001, -83.000}},
		},
		{
			name:  "Charlotte Avenue",
			nodes: [][2]float64{{40.001, -83.000}, {40.002, -83.000}},
		},
		{
			name:     "Charlotte Avenue",
			maxSpeed: forwardSpeed,
			nodes:    [][2]float64{{40.002, -83.000}, {40.003, -83.000}},
		},
	})
}

func guessForCurrent(t *testing.T, offline Offline, currentIndex int) SpeedLimitGuess {
	t.Helper()
	ways, err := offline.Ways()
	if err != nil {
		t.Fatalf("could not read ways: %v", err)
	}
	current := CurrentWay{
		Way:   ways.At(currentIndex),
		OnWay: OnWayResult{OnWay: true, IsForward: true},
	}
	return ComputeSpeedLimitGuess(current, offline)
}

func TestGuessBackwardOnly(t *testing.T) {
	guess := guessForCurrent(t, guessOffline(t, mph35, 0), 1)
	if guess.Source != "backward" {
		t.Fatalf("expected source backward, got %q", guess.Source)
	}
	if math.Abs(guess.Speedlimit-mph35) > 1e-9 || math.Abs(guess.BackwardValue-mph35) > 1e-9 {
		t.Errorf("expected 35 mph backward guess, got %+v", guess)
	}
	if guess.ForwardValue != 0 {
		t.Errorf("expected no forward value, got %+v", guess)
	}
	if guess.BackwardDistance != 0 {
		t.Errorf("expected zero distance to the adjacent tagged way, got %+v", guess)
	}
}

func TestGuessForwardOnly(t *testing.T) {
	guess := guessForCurrent(t, guessOffline(t, 0, mph40), 1)
	if guess.Source != "forward" {
		t.Fatalf("expected source forward, got %q", guess.Source)
	}
	if math.Abs(guess.Speedlimit-mph40) > 1e-9 {
		t.Errorf("expected 40 mph forward guess, got %+v", guess)
	}
}

func TestGuessBothEqual(t *testing.T) {
	guess := guessForCurrent(t, guessOffline(t, mph35, mph35), 1)
	if guess.Source != "both" {
		t.Fatalf("expected source both, got %q", guess.Source)
	}
	if math.Abs(guess.Speedlimit-mph35) > 1e-9 {
		t.Errorf("expected 35 mph guess, got %+v", guess)
	}
}

// Conflicting values: the backward (already passed) signage wins; the forward
// change surfaces separately via NextMapSpeedLimit.
func TestGuessBothDifferingPrefersBackward(t *testing.T) {
	guess := guessForCurrent(t, guessOffline(t, mph35, mph40), 1)
	if guess.Source != "backward" {
		t.Fatalf("expected source backward on conflict, got %q", guess.Source)
	}
	if math.Abs(guess.Speedlimit-mph35) > 1e-9 {
		t.Errorf("expected the backward 35 mph value to win, got %+v", guess)
	}
	if math.Abs(guess.ForwardValue-mph40) > 1e-9 {
		t.Errorf("expected the forward 40 mph value to still be reported, got %+v", guess)
	}
}

func TestGuessSkippedWithoutNameOrRef(t *testing.T) {
	offline := buildOffline(t, []testWay{
		{
			name:     "Charlotte Avenue",
			maxSpeed: mph35,
			nodes:    [][2]float64{{40.000, -83.000}, {40.001, -83.000}},
		},
		{
			// Unnamed, no ref: no continuity to walk along.
			nodes: [][2]float64{{40.001, -83.000}, {40.002, -83.000}},
		},
	})
	guess := guessForCurrent(t, offline, 1)
	if guess != (SpeedLimitGuess{}) {
		t.Errorf("expected empty guess for a way with neither name nor ref, got %+v", guess)
	}
}

func TestGuessSkippedWhenCurrentWayTagged(t *testing.T) {
	offline := guessOffline(t, mph35, mph40)
	ways, _ := offline.Ways()
	// Use the tagged first way as the current way.
	current := CurrentWay{Way: ways.At(0), OnWay: OnWayResult{OnWay: true, IsForward: true}}
	guess := ComputeSpeedLimitGuess(current, offline)
	if guess != (SpeedLimitGuess{}) {
		t.Errorf("expected empty guess when the current way has a tagged limit, got %+v", guess)
	}
}

// A tagged way further than GUESS_MAX_SEARCH_DISTANCE along the road must not
// be used.
func TestGuessDistanceCap(t *testing.T) {
	offline := buildOffline(t, []testWay{
		{
			name:     "Charlotte Avenue",
			maxSpeed: mph35,
			// ~4400 m long: past the 3000 m cap once walked over.
			nodes: [][2]float64{{39.956, -83.000}, {39.960, -83.000}},
		},
		{
			// Untagged filler between the tagged way and the current way.
			name:  "Charlotte Avenue",
			nodes: [][2]float64{{39.960, -83.000}, {40.000, -83.000}},
		},
		{
			name:  "Charlotte Avenue",
			nodes: [][2]float64{{40.000, -83.000}, {40.001, -83.000}},
		},
	})
	guess := guessForCurrent(t, offline, 2)
	if guess.BackwardValue != 0 || guess.Source != "" {
		t.Errorf("expected no guess beyond the 3000 m search cap, got %+v", guess)
	}
}

// Same-ref continuity works when names are absent, and directional tags on the
// found way are preferred for the travel direction on that way.
func TestGuessRefContinuityAndDirectionalTag(t *testing.T) {
	offline := buildOffline(t, []testWay{
		{
			ref: "US 70",
			// Travel toward the current way is "forward" on this way (first ->
			// last node ends at the shared node), so maxspeed:forward applies.
			maxSpeedForward:  mph35,
			maxSpeedBackward: mph40,
			nodes:            [][2]float64{{40.000, -83.000}, {40.001, -83.000}},
		},
		{
			ref:   "US 70",
			nodes: [][2]float64{{40.001, -83.000}, {40.002, -83.000}},
		},
	})
	guess := guessForCurrent(t, offline, 1)
	if guess.Source != "backward" {
		t.Fatalf("expected source backward via ref continuity, got %q", guess.Source)
	}
	if math.Abs(guess.Speedlimit-mph35) > 1e-9 {
		t.Errorf("expected the travel-direction (forward) 35 mph tag, got %+v", guess)
	}
}
