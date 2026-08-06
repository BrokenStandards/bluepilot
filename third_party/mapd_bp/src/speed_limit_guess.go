package main

// Speed-limit gap guessing: when the matched way has no tagged maxspeed, walk
// the road graph along same-name/same-ref continuations in both directions and
// report the nearest tagged values. The guess is published on its own mem param
// (MapSpeedLimitGuess) and never touches MapSpeedLimit, so consumers can treat
// it with lower confidence than a tagged limit.

const (
	// How far along the road (sum of intermediate way lengths) each directional
	// walk may travel before giving up. Beyond this the signage zone the car is
	// in can no longer be assumed to match the found tag.
	GUESS_MAX_SEARCH_DISTANCE = 3000.0 // meters
	// Hop cap so dense node-split roads (many tiny ways) cannot make the walk
	// unbounded in work even when the distance cap is not yet reached.
	GUESS_MAX_WAY_HOPS = 40
)

type SpeedLimitGuess struct {
	Speedlimit       float64 `json:"speedlimit"`
	Source           string  `json:"source"`
	BackwardValue    float64 `json:"backward_value"`
	BackwardDistance float64 `json:"backward_distance"`
	ForwardValue     float64 `json:"forward_value"`
	ForwardDistance  float64 `json:"forward_distance"`
}

// effectiveMaxSpeed is the directional maxspeed for travel direction isForward
// on the way, falling back to the generic maxspeed tag.
func effectiveMaxSpeed(way Way, isForward bool) float64 {
	if isForward && way.MaxSpeedForward() > 0 {
		return way.MaxSpeedForward()
	}
	if !isForward && way.MaxSpeedBackward() > 0 {
		return way.MaxSpeedBackward()
	}
	return way.MaxSpeed()
}

func namesOrRefsMatch(way Way, name string, ref string) bool {
	wName, err := way.Name()
	if err == nil && len(name) > 0 && wName == name {
		return true
	}
	wRef, err := way.Ref()
	if err == nil && len(ref) > 0 && wRef == ref {
		return true
	}
	return false
}

// walkForTaggedSpeedLimit walks the road graph from startWay along
// same-name/same-ref continuations, looking for the nearest way with a tagged
// maxspeed. walkForward is the traversal direction of the FIRST step relative
// to startWay's node order. reverseWalk marks the backward (upstream) walk: the
// hypothetical travel direction on visited ways is then opposite to the walk
// direction, and the oneway drivability rejection is skipped — we are sampling
// the road's signage zone, not planning a route.
// Returns the tagged value and the along-road distance from startWay's
// boundary node to the near end of the tagged way (0 when adjacent).
func walkForTaggedSpeedLimit(startWay Way, offline Offline, walkForward bool, reverseWalk bool, name string, ref string) (float64, float64) {
	cur := startWay
	curForward := walkForward
	dist := 0.0

	for hops := 0; hops < GUESS_MAX_WAY_HOPS; hops++ {
		nodes, err := cur.Nodes()
		if err != nil || nodes.Len() < 2 {
			return 0, 0
		}

		var matchNode Coordinates
		var bearingNode Coordinates
		if curForward {
			matchNode = nodes.At(nodes.Len() - 1)
			bearingNode = nodes.At(nodes.Len() - 2)
		} else {
			matchNode = nodes.At(0)
			bearingNode = nodes.At(1)
		}

		if !PointInBox(matchNode.Latitude(), matchNode.Longitude(), offline.MinLat()-offline.Overlap(), offline.MinLon()-offline.Overlap(), offline.MaxLat()+offline.Overlap(), offline.MaxLon()+offline.Overlap()) {
			return 0, 0
		}

		matchingWays, err := MatchingWays(cur, offline, matchNode)
		if err != nil {
			return 0, 0
		}

		var next Way
		found := false
		for _, mWay := range matchingWays {
			if !namesOrRefsMatch(mWay, name, ref) {
				continue
			}
			if isUTurn(mWay, matchNode, bearingNode) {
				continue
			}
			nextForward := NextIsForward(mWay, matchNode)
			if !reverseWalk && !nextForward && mWay.OneWay() {
				continue
			}
			next = mWay
			found = true
			break
		}
		if !found {
			return 0, 0
		}

		nextForward := NextIsForward(next, matchNode)
		travelForward := nextForward
		if reverseWalk {
			// Walking upstream: a car that would later reach us drives the way
			// toward matchNode, i.e. against our walk traversal.
			travelForward = !nextForward
		}

		if v := effectiveMaxSpeed(next, travelForward); v > 0 {
			return v, dist
		}

		wayLength, err := calculateWayDistance(next)
		if err != nil {
			return 0, 0
		}
		dist += wayLength
		if dist > GUESS_MAX_SEARCH_DISTANCE {
			return 0, 0
		}
		cur = next
		curForward = nextForward
	}
	return 0, 0
}

// ComputeSpeedLimitGuess builds the MapSpeedLimitGuess payload for the current
// way. It is cached by the caller and recomputed only when the current way
// changes or the offline data reloads.
func ComputeSpeedLimitGuess(currentWay CurrentWay, offline Offline) SpeedLimitGuess {
	guess := SpeedLimitGuess{}
	way := currentWay.Way
	if !way.HasNodes() {
		return guess
	}

	isForward := currentWay.OnWay.IsForward
	if effectiveMaxSpeed(way, isForward) > 0 {
		// The way has a real tagged limit; no guessing.
		return guess
	}

	name, _ := way.Name()
	ref, _ := way.Ref()
	if len(name) == 0 && len(ref) == 0 {
		// Without name or ref there is no continuity to walk along.
		return guess
	}

	guess.ForwardValue, guess.ForwardDistance = walkForTaggedSpeedLimit(way, offline, isForward, false, name, ref)
	guess.BackwardValue, guess.BackwardDistance = walkForTaggedSpeedLimit(way, offline, !isForward, true, name, ref)

	// The backward value wins on conflict: the car most recently passed that
	// signage zone, and a differing forward value still surfaces through the
	// existing NextMapSpeedLimit mechanism since MapSpeedLimit stays 0.
	switch {
	case guess.BackwardValue > 0 && guess.ForwardValue > 0 && guess.BackwardValue == guess.ForwardValue:
		guess.Speedlimit = guess.BackwardValue
		guess.Source = "both"
	case guess.BackwardValue > 0:
		guess.Speedlimit = guess.BackwardValue
		guess.Source = "backward"
	case guess.ForwardValue > 0:
		guess.Speedlimit = guess.ForwardValue
		guess.Source = "forward"
	}
	return guess
}
