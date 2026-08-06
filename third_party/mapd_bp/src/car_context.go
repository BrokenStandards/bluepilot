package main

// Divergence-based rematch: compare what the car is physically doing (yaw-rate
// derived curvature published by osm_map_data on MapdCarContext) with the local
// curvature of the matched way. When they disagree for several consecutive
// ticks the sticky current-way match is likely wrong (e.g. matched to the
// opposite carriageway or a parallel road), so a full candidate selection is
// run with an extra score term rewarding candidates whose geometry matches the
// car. The rematch NEVER forces a switch: if no candidate strictly outscores
// the current way, the current match is kept.

import (
	"encoding/json"
	"math"
)

const (
	// Curvature disagreement (1/m) between car and matched way that counts as a
	// divergence tick. 0.004 1/m at city speed is a clearly felt steering
	// difference, well above yaw-rate sensor noise.
	DIVERGENCE_CURVATURE_DELTA = 0.004
	// At least one of the two curvatures must exceed this (1/m), so two
	// nearly-straight signals cannot accumulate divergence ticks from noise.
	DIVERGENCE_MIN_CURVATURE = 0.005
	// Consecutive 1 Hz ticks of divergence required before triggering a
	// rematch; filters transient disagreement (lane changes, GPS jumps).
	DIVERGENCE_TRIGGER_TICKS = 3
	// Bound of the curvature-match score contribution during a rematch, chosen
	// to be able to override the same-name (+30) stickiness bonus but not the
	// hierarchy spread, so a freeway never loses to a service road on
	// curvature alone.
	CURVATURE_MATCH_MAX_SCORE = 30.0
	// Curvature difference (1/m) at which the curvature-match score saturates
	// at -CURVATURE_MATCH_MAX_SCORE (difference 0 scores +CURVATURE_MATCH_MAX_SCORE).
	CURVATURE_MATCH_SATURATION_DELTA = 0.01
)

type CarContext struct {
	Enabled          bool    `json:"enabled"`
	VEgo             float64 `json:"v_ego"`
	YawRate          float64 `json:"yaw_rate"`
	Curvature        float64 `json:"curvature"`
	DesiredCurvature float64 `json:"desired_curvature"`
}

func ReadCarContext() CarContext {
	carContext := CarContext{}
	data, err := GetParam(MAPD_CAR_CONTEXT)
	if err != nil {
		// Missing param means the Python side is not publishing; the zero value
		// (Enabled false) disables all divergence logic.
		return carContext
	}
	err = json.Unmarshal(data, &carContext)
	if err != nil {
		return CarContext{}
	}
	return carContext
}

// WayLocalCurvature computes the way's curvature from the node triple nearest
// the given position. Returns false when the way has fewer than 3 nodes.
func WayLocalCurvature(way Way, pos Position) (float64, bool) {
	nodes, err := way.Nodes()
	if err != nil || nodes.Len() < 3 {
		return 0, false
	}

	nearest := 0
	minDistance := math.MaxFloat64
	latRad := pos.Latitude * TO_RADIANS
	lonRad := pos.Longitude * TO_RADIANS
	for i := 0; i < nodes.Len(); i++ {
		node := nodes.At(i)
		distance := DistanceToPoint(latRad, lonRad, node.Latitude()*TO_RADIANS, node.Longitude()*TO_RADIANS)
		if distance < minDistance {
			minDistance = distance
			nearest = i
		}
	}

	center := nearest
	if center == 0 {
		center = 1
	}
	if center == nodes.Len()-1 {
		center = nodes.Len() - 2
	}
	a := nodes.At(center - 1)
	b := nodes.At(center)
	c := nodes.At(center + 1)
	curv, _, _ := GetCurvature(a.Latitude(), a.Longitude(), b.Latitude(), b.Longitude(), c.Latitude(), c.Longitude())
	return curv, true
}

// UpdateDivergence advances the divergence tick counter and reports whether a
// rematch should run this tick. Must be called after state.Position is updated
// but before GetCurrentWay, so it compares the car against last tick's match.
func UpdateDivergence(state *State) bool {
	if !state.CarContext.Enabled || !state.CurrentWay.Way.HasNodes() {
		state.DivergenceTicks = 0
		return false
	}

	wayCurv, ok := WayLocalCurvature(state.CurrentWay.Way, state.Position)
	if !ok {
		state.DivergenceTicks = 0
		return false
	}

	carCurv := math.Abs(state.CarContext.Curvature)
	mapCurv := math.Abs(wayCurv)
	diverged := math.Abs(carCurv-mapCurv) > DIVERGENCE_CURVATURE_DELTA &&
		(carCurv > DIVERGENCE_MIN_CURVATURE || mapCurv > DIVERGENCE_MIN_CURVATURE)

	if diverged {
		state.DivergenceTicks++
	} else {
		state.DivergenceTicks = 0
	}

	if state.DivergenceTicks >= DIVERGENCE_TRIGGER_TICKS {
		// Require a fresh streak before the next trigger so a failed rematch
		// does not re-run the full-map scan every tick.
		state.DivergenceTicks = 0
		return true
	}
	return false
}

// curvatureMatchScore rewards candidates whose local geometry matches the
// car's measured curvature magnitude. Linear from +CURVATURE_MATCH_MAX_SCORE
// at difference 0 down to -CURVATURE_MATCH_MAX_SCORE at
// CURVATURE_MATCH_SATURATION_DELTA, clamped there.
func curvatureMatchScore(way Way, pos Position, carCurvature float64) float64 {
	wayCurv, ok := WayLocalCurvature(way, pos)
	if !ok {
		return 0
	}
	diff := math.Abs(math.Abs(carCurvature) - math.Abs(wayCurv))
	score := CURVATURE_MATCH_MAX_SCORE * (1 - 2*diff/CURVATURE_MATCH_SATURATION_DELTA)
	return math.Max(-CURVATURE_MATCH_MAX_SCORE, math.Min(CURVATURE_MATCH_MAX_SCORE, score))
}
