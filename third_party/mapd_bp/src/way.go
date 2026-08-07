package main

import (
	"math"
	"strings"
	"time"

	"capnproto.org/go/capnp/v3"
	"github.com/pkg/errors"
)

var MIN_WAY_DIST = 500 // meters. how many meters to look ahead before stopping gathering next ways.

type RoadContext int

const (
	CONTEXT_FREEWAY RoadContext = iota
	CONTEXT_CITY
	CONTEXT_UNKNOWN
)

// Road type detection and priorities
var LANE_COUNT_PRIORITY = map[uint8]int{
	8: 110, // Major freeway
	6: 100, // Freeway
	5: 95,
	4: 90, // Major arterial
	3: 70, // Arterial
	2: 50, // Collector/local
	1: 40, // Local street
	0: 30, // Unknown
}

// Highway hierarchy ranking
var HIGHWAY_RANK = map[string]int{
	"motorway":       0,
	"motorway_link":  1,
	"trunk":          10,
	"trunk_link":     11,
	"primary":        20,
	"primary_link":   21,
	"secondary":      30,
	"secondary_link": 31,
	"tertiary":       40,
	"tertiary_link":  41,
	"unclassified":   50,
	"residential":    60,
	"living_street":  61,
}

// Bearing alignment thresholds
const ACCEPTABLE_BEARING_DELTA_SIN = 0.7071067811865475 // sin(45°) - max acceptable bearing mismatch

// A next-way candidate is a U-turn when the ROAD-AXIS bearing of the
// continuation onto it reverses the road-axis bearing of the approach by more
// than 150°. cos(150°) = -0.866; anything below that is a direction reversal
// onto e.g. the opposite carriageway of a divided road, which chains must
// never follow.
const U_TURN_BEARING_DELTA_COS = -0.866

// U_TURN_AXIS_DIST is the along-road distance (meters) each side of the
// junction is walked to establish its road-axis bearing before the U-turn
// test. Single adjacent-segment bearings under-measure the reversal at
// divided-carriageway crossover junctions: the ~9 m median-crossover jog
// segments rotate ~30-45° toward each other, so a true 180° carriageway
// reversal can measure only ~117-127° and slip past the 150° threshold. 30 m
// is long enough to swallow the jog (crossover jogs are of the order of a
// road width, ~10-15 m) while staying local enough that ordinary same-name
// corners keep their true turn angle.
const U_TURN_AXIS_DIST = 30.0

// U_TURN_AXIS_MAX_HOPS bounds how many additional connected same-name ways
// the approach-axis walk may continue into when the from-way itself is
// shorter than U_TURN_AXIS_DIST (dual carriageways are often chopped into
// tiny ways right at crossover junctions).
const U_TURN_AXIS_MAX_HOPS = 2

type OnWayResult struct {
	OnWay     bool
	Distance  DistanceResult
	IsForward bool
}

type WayCandidate struct {
	Way              Way
	OnWayResult      OnWayResult
	BearingAlignment float64 // sin(bearing_delta) - lower is better
	DistanceToWay    float64
	HierarchyRank    int
	Context          RoadContext
}

type DistanceResult struct {
	LineStart Coordinates
	LineEnd   Coordinates
	Distance  float64
}

// Updated CurrentWay struct with stability fields
type CurrentWay struct {
	Way               Way
	Distance          DistanceResult
	OnWay             OnWayResult
	StartPosition     Coordinates
	EndPosition       Coordinates
	ConfidenceCounter int
	LastChangeTime    time.Time
	StableDistance    float64
}

type NextWayResult struct {
	Way           Way
	IsForward     bool
	StartPosition Coordinates
	EndPosition   Coordinates
}

func OnWay(way Way, pos Position, extended bool) (OnWayResult, error) {
	res := OnWayResult{}
	if pos.Latitude < way.MaxLat()+PADDING && pos.Latitude > way.MinLat()-PADDING && pos.Longitude < way.MaxLon()+PADDING && pos.Longitude > way.MinLon()-PADDING {
		d, err := DistanceToWay(pos, way)
		res.Distance = d
		if err != nil {
			res.OnWay = false
			return res, errors.Wrap(err, "could not get distance to way")
		}
		lanes := way.Lanes()
		if lanes == 0 {
			lanes = 2
		}
		road_width_estimate := float64(lanes) * LANE_WIDTH
		max_dist := 5 + road_width_estimate
		if extended {
			max_dist = max_dist * 2
		}

		context := determineRoadContext(way, pos)
		if context == CONTEXT_FREEWAY {
			max_dist = max_dist * 1.5
		} else if context == CONTEXT_CITY {
			max_dist = max_dist * 0.8
		}

		if d.Distance < max_dist {
			res.OnWay = true
			res.IsForward = IsForward(d.LineStart, d.LineEnd, pos.Bearing)
			if !res.IsForward && way.OneWay() {
				res.OnWay = false
			}
			return res, nil
		}
	}
	res.OnWay = false
	return res, nil
}

func determineRoadContext(way Way, pos Position) RoadContext {
	lanes := way.Lanes()
	name, _ := way.Name()
	ref, _ := way.Ref()

	if isFreeway(way) || lanes >= 4 {
		return CONTEXT_FREEWAY
	}

	nameUpper := strings.ToUpper(name)
	if lanes <= 3 && (strings.Contains(nameUpper, "STREET") ||
		strings.Contains(nameUpper, "AVENUE") ||
		strings.Contains(nameUpper, "BOULEVARD") ||
		strings.Contains(nameUpper, "ROAD") ||
		len(ref) == 0) {
		return CONTEXT_CITY
	}

	return CONTEXT_UNKNOWN
}

func isFreeway(way Way) bool {
	lanes := way.Lanes()
	name, _ := way.Name()
	ref, _ := way.Ref()

	if lanes >= 6 {
		return true
	}

	nameUpper := strings.ToUpper(name)
	refUpper := strings.ToUpper(ref)

	if strings.Contains(nameUpper, "INTERSTATE") ||
		strings.Contains(nameUpper, "FREEWAY") ||
		strings.Contains(nameUpper, "EXPRESSWAY") ||
		strings.Contains(nameUpper, "PARKWAY") ||
		strings.HasPrefix(refUpper, "I-") ||
		strings.HasPrefix(refUpper, "I ") ||
		(lanes >= 4 && len(ref) > 0 && !strings.Contains(nameUpper, "STREET")) {
		return true
	}

	return false
}

// Get highway hierarchy rank for a way
func getHighwayRank(way Way) int {
	name, _ := way.Name()
	ref, _ := way.Ref()
	lanes := way.Lanes()

	// Infer highway type from characteristics
	if isFreeway(way) {
		if lanes >= 6 {
			return HIGHWAY_RANK["motorway"]
		}
		return HIGHWAY_RANK["trunk"]
	}

	nameUpper := strings.ToUpper(name)
	refUpper := strings.ToUpper(ref)

	// Primary roads (usually have ref numbers)
	if len(ref) > 0 && !strings.Contains(nameUpper, "STREET") {
		if strings.HasPrefix(refUpper, "US-") || strings.HasPrefix(refUpper, "SR-") {
			return HIGHWAY_RANK["primary"]
		}
		return HIGHWAY_RANK["secondary"]
	}

	// Local roads
	if strings.Contains(nameUpper, "STREET") ||
		strings.Contains(nameUpper, "AVENUE") ||
		strings.Contains(nameUpper, "ROAD") {
		return HIGHWAY_RANK["residential"]
	}

	// Default to unclassified
	return HIGHWAY_RANK["unclassified"]
}

func calculateBearingAlignment(way Way, pos Position) (float64, error) {
	d, err := DistanceToWay(pos, way)
	if err != nil {
		return 1.0, err
	}

	startLat := d.LineStart.Latitude()
	startLon := d.LineStart.Longitude()
	endLat := d.LineEnd.Latitude()
	endLon := d.LineEnd.Longitude()

	wayBearing := Bearing(startLat, startLon, endLat, endLon)

	// Calculate bearing delta
	delta := math.Abs(pos.Bearing*TO_RADIANS - wayBearing)

	// Normalize to 0-π range
	if delta > math.Pi {
		delta = 2*math.Pi - delta
	}
	return math.Sin(delta), nil
}

// scoreWayCandidate scores a candidate for full current-way selection. Returns
// ok=false when the car is not on the way at all. useCarCurvature adds the
// divergence-rematch score term (see curvatureMatchScore).
func scoreWayCandidate(way Way, pos Position, currentWay Way, useCarCurvature bool, carCurvature float64) (float64, bool) {
	onWay, err := OnWay(way, pos, false)
	if err != nil || !onWay.OnWay {
		return 0, false
	}

	score := float64(0)

	hierarchyRank := getHighwayRank(way)
	score += float64(100 - hierarchyRank)

	bearingAlignment, err := calculateBearingAlignment(way, pos)
	if err == nil {
		score += (1.0 - bearingAlignment) * 50
	}

	score -= onWay.Distance.Distance * 0.1

	if currentWay.HasNodes() {
		currentName, _ := currentWay.Name()
		currentRef, _ := currentWay.Ref()
		wayName, _ := way.Name()
		wayRef, _ := way.Ref()

		if len(currentName) > 0 && currentName == wayName {
			score += 30.0
		}
		if len(currentRef) > 0 && currentRef == wayRef {
			score += 25.0
		}
	}

	if useCarCurvature {
		score += curvatureMatchScore(way, pos, carCurvature)
	}

	return score, true
}

func selectBestWayAdvanced(possibleWays []Way, pos Position, currentWay Way, context RoadContext, gpsAccuracy float64) Way {
	if len(possibleWays) == 0 {
		return Way{}
	}
	if len(possibleWays) == 1 {
		return possibleWays[0]
	}

	bestWay := possibleWays[0]
	bestScore := float64(-1000)

	for _, way := range possibleWays {
		score, ok := scoreWayCandidate(way, pos, currentWay, false, 0)
		if !ok {
			continue
		}

		if score > bestScore {
			bestScore = score
			bestWay = way
		}
	}

	return bestWay
}

func isSameWay(a Way, b Way) bool {
	return a.MinLat() == b.MinLat() && a.MaxLat() == b.MaxLat() && a.MinLon() == b.MinLon() && a.MaxLon() == b.MaxLon()
}

// rematchDivergedWay runs a full candidate selection with the car-curvature
// score term. It returns ok=true only when some other way STRICTLY outscores
// the current way under the same scoring — otherwise the caller keeps the
// current match (fail safe).
func rematchDivergedWay(currentWay Way, offline Offline, pos Position, carCurvature float64) (Way, bool) {
	possibleWays, err := getPossibleWays(offline, pos)
	if err != nil || len(possibleWays) == 0 {
		return Way{}, false
	}

	currentScore := math.Inf(-1)
	if currentWay.HasNodes() {
		if score, ok := scoreWayCandidate(currentWay, pos, currentWay, true, carCurvature); ok {
			currentScore = score
		}
	}

	bestScore := currentScore
	bestWay := Way{}
	found := false
	for _, way := range possibleWays {
		if isSameWay(way, currentWay) {
			continue
		}
		score, ok := scoreWayCandidate(way, pos, currentWay, true, carCurvature)
		if !ok {
			continue
		}
		if score > bestScore {
			bestScore = score
			bestWay = way
			found = true
		}
	}

	return bestWay, found
}

func getRoadPriority(way Way, context RoadContext) int {
	lanes := way.Lanes()
	name, _ := way.Name()
	ref, _ := way.Ref()

	priority := LANE_COUNT_PRIORITY[lanes]
	if priority == 0 {
		priority = 30
	}

	switch context {
	case CONTEXT_FREEWAY:
		if isFreeway(way) {
			priority += 30
		}
		nameUpper := strings.ToUpper(name)
		if strings.Contains(nameUpper, "STREET") ||
			strings.Contains(nameUpper, "AVENUE") {
			priority -= 40
		}

	case CONTEXT_CITY:
		if isFreeway(way) {
			priority += 10
		}
		nameUpper := strings.ToUpper(name)
		if strings.Contains(nameUpper, "SERVICE") {
			priority -= 5
		}
		// Boost local street names in city context
		if strings.Contains(nameUpper, "STREET") ||
			strings.Contains(nameUpper, "AVENUE") ||
			strings.Contains(nameUpper, "ROAD") {
			priority += 5
		}

	case CONTEXT_UNKNOWN:
		if isFreeway(way) {
			priority += 20
		}
	}

	if len(ref) > 0 {
		priority += 10
	}

	return priority
}

func DistanceToWay(pos Position, way Way) (DistanceResult, error) {
	res := DistanceResult{}
	var minNodeStart Coordinates
	var minNodeEnd Coordinates
	minDistance := math.MaxFloat64
	nodes, err := way.Nodes()
	if err != nil {
		return res, errors.Wrap(err, "could not read way nodes")
	}
	if nodes.Len() < 2 {
		return res, nil
	}

	latRad := pos.Latitude * TO_RADIANS
	lonRad := pos.Longitude * TO_RADIANS
	for i := 0; i < nodes.Len()-1; i++ {
		nodeStart := nodes.At(i)
		nodeEnd := nodes.At(i + 1)
		lineLat, lineLon := PointOnLine(nodeStart.Latitude(), nodeStart.Longitude(), nodeEnd.Latitude(), nodeEnd.Longitude(), pos.Latitude, pos.Longitude)
		distance := DistanceToPoint(latRad, lonRad, lineLat*TO_RADIANS, lineLon*TO_RADIANS)
		if distance < minDistance {
			minDistance = distance
			minNodeStart = nodeStart
			minNodeEnd = nodeEnd
		}
	}
	res.Distance = minDistance
	res.LineStart = minNodeStart
	res.LineEnd = minNodeEnd
	return res, nil
}

func GetWayStartEnd(way Way, isForward bool) (Coordinates, Coordinates) {
	if !way.HasNodes() {
		return Coordinates{}, Coordinates{}
	}

	nodes, err := way.Nodes()
	if err != nil {
		logde(errors.Wrap(err, "could not read way nodes"))
		return Coordinates{}, Coordinates{}
	}

	if nodes.Len() == 0 {
		return Coordinates{}, Coordinates{}
	}

	if nodes.Len() == 1 {
		return nodes.At(0), nodes.At(0)
	}

	if isForward {
		return nodes.At(0), nodes.At(nodes.Len() - 1)
	}
	return nodes.At(nodes.Len() - 1), nodes.At(0)
}

func GetCurrentWay(currentWay CurrentWay, nextWays []NextWayResult, offline Offline, pos Position, lastPos Position, gpsAccuracy float64, forceRematch bool, carCurvature float64) (CurrentWay, error) {
	currentContext := CONTEXT_UNKNOWN
	if currentWay.Way.HasNodes() {
		currentContext = determineRoadContext(currentWay.Way, pos)
	}

	// Divergence-triggered rematch: bypass the sticky-match branch below and
	// re-run full candidate selection with the car-curvature score term. Only
	// switches when another way strictly outscores the current one.
	if forceRematch {
		if way, ok := rematchDivergedWay(currentWay.Way, offline, pos, carCurvature); ok {
			onWay, err := OnWay(way, pos, false)
			if err == nil && onWay.OnWay {
				start, end := GetWayStartEnd(way, onWay.IsForward)
				return CurrentWay{
					Way:               way,
					Distance:          onWay.Distance,
					OnWay:             onWay,
					StartPosition:     start,
					EndPosition:       end,
					ConfidenceCounter: 1,
					LastChangeTime:    time.Now(),
					StableDistance:    onWay.Distance.Distance,
				}, nil
			}
		}
		// No strictly better candidate: fail safe to the normal (sticky) flow.
	}

	if currentWay.Way.HasNodes() {
		onWay, err := OnWay(currentWay.Way, pos, false)
		if err == nil && onWay.OnWay {
			stickThreshold := 15.0
			if currentContext == CONTEXT_FREEWAY {
				stickThreshold = 20.0
			} else if currentContext == CONTEXT_CITY {
				stickThreshold = 10.0
			}

			if onWay.Distance.Distance < stickThreshold {
				newStableDistance := onWay.Distance.Distance

				start, end := GetWayStartEnd(currentWay.Way, onWay.IsForward)
				return CurrentWay{
					Way:               currentWay.Way,
					Distance:          onWay.Distance,
					OnWay:             onWay,
					StartPosition:     start,
					EndPosition:       end,
					ConfidenceCounter: currentWay.ConfidenceCounter + 1,
					LastChangeTime:    currentWay.LastChangeTime,
					StableDistance:    newStableDistance,
				}, nil
			}
		}
	}

	for _, nextWay := range nextWays {
		onWay, err := OnWay(nextWay.Way, pos, false)
		if err == nil && onWay.OnWay {
			start, end := GetWayStartEnd(nextWay.Way, onWay.IsForward)
			return CurrentWay{
				Way:               nextWay.Way,
				Distance:          onWay.Distance,
				OnWay:             onWay,
				StartPosition:     start,
				EndPosition:       end,
				ConfidenceCounter: 1,
				LastChangeTime:    time.Now(),
				StableDistance:    onWay.Distance.Distance,
			}, nil
		}
	}

	possibleWays, err := getPossibleWays(offline, pos)
	if err == nil && len(possibleWays) > 0 {
		selectedWay := selectBestWayAdvanced(possibleWays, pos, currentWay.Way, currentContext, gpsAccuracy)
		if selectedWay.HasNodes() {
			selectedOnWay, err := OnWay(selectedWay, pos, false)
			if err == nil && selectedOnWay.OnWay {
				start, end := GetWayStartEnd(selectedWay, selectedOnWay.IsForward)
				return CurrentWay{
					Way:               selectedWay,
					Distance:          selectedOnWay.Distance,
					OnWay:             selectedOnWay,
					StartPosition:     start,
					EndPosition:       end,
					ConfidenceCounter: 1,
					LastChangeTime:    time.Now(),
					StableDistance:    selectedOnWay.Distance.Distance,
				}, nil
			}
		}
	}

	if currentWay.Way.HasNodes() {
		onWay, err := OnWay(currentWay.Way, pos, true)
		if err == nil && onWay.OnWay {
			start, end := GetWayStartEnd(currentWay.Way, onWay.IsForward)
			return CurrentWay{
				Way:               currentWay.Way,
				Distance:          onWay.Distance,
				OnWay:             onWay,
				StartPosition:     start,
				EndPosition:       end,
				ConfidenceCounter: currentWay.ConfidenceCounter,
				LastChangeTime:    currentWay.LastChangeTime,
				StableDistance:    currentWay.StableDistance,
			}, nil
		}
	}

	return CurrentWay{}, errors.New("could not find a current way")
}

func getPossibleWays(offline Offline, pos Position) ([]Way, error) {
	possibleWays := []Way{}
	ways, err := offline.Ways()
	if err != nil {
		return possibleWays, errors.Wrap(err, "could not get other ways")
	}

	for i := 0; i < ways.Len(); i++ {
		way := ways.At(i)
		onWay, err := OnWay(way, pos, false)
		logde(errors.Wrap(err, "Could not check if on way"))
		if onWay.OnWay {
			possibleWays = append(possibleWays, way)
		}
	}
	return possibleWays, nil
}

func IsForward(lineStart Coordinates, lineEnd Coordinates, bearing float64) bool {
	startLat := lineStart.Latitude()
	startLon := lineStart.Longitude()
	endLat := lineEnd.Latitude()
	endLon := lineEnd.Longitude()

	wayBearing := Bearing(startLat, startLon, endLat, endLon)
	bearingDelta := math.Abs(bearing*TO_RADIANS - wayBearing)
	return math.Cos(bearingDelta) >= 0
}

// MatchingWays returns every way other than currentWay that starts or ends
// exactly at matchNode, in tile order (several callers take the first match).
//
// BluePilot: this used to be a linear scan over all ways, resolving every
// way's node list to read two endpoints — 3.24 ms per call on the 11037-way
// Nashville tile, and it is called once per graph hop (NextWay, the
// speed-limit-guess walk, uTurnInAxisPoint): p95 22 and up to 105 calls per
// 1 Hz tick, 70.66% of mapd's total CPU. It is now a lookup in an endpoint
// index built once per tile load (see tile_cache.go). The index is keyed on
// the exact float64 (lat, lon) pair this function compared with ==, admits
// ways under the same HasNodes/Len>=2 rules, and is built in tile order, so
// the returned slice is identical — verified over 19,589 cross-checked calls
// on the recorded corpus.
// End BluePilot
func MatchingWays(currentWay Way, offline Offline, matchNode Coordinates) ([]Way, error) {
	matchingWays := []Way{}
	ways, err := offline.Ways()
	if err != nil {
		return matchingWays, errors.Wrap(err, "could not read ways from offline")
	}

	byEnd, err := endpointIndexFor(offline, ways)
	if err != nil {
		return matchingWays, err
	}

	for _, i := range byEnd[endpointKey{matchNode.Latitude(), matchNode.Longitude()}] {
		w := ways.At(int(i))
		if w.MinLat() == currentWay.MinLat() && w.MaxLat() == currentWay.MaxLat() && w.MinLon() == currentWay.MinLon() && w.MaxLon() == currentWay.MaxLon() {
			continue
		}
		matchingWays = append(matchingWays, w)
	}

	return matchingWays, nil
}

func NextIsForward(nextWay Way, matchNode Coordinates) bool {
	if !nextWay.HasNodes() {
		return true
	}
	nodes, err := nextWay.Nodes()
	if err != nil || nodes.Len() < 2 {
		logde(errors.Wrap(err, "could not read next way nodes"))
		return true
	}

	lastNode := nodes.At(nodes.Len() - 1)
	if lastNode.Latitude() == matchNode.Latitude() && lastNode.Longitude() == matchNode.Longitude() {
		return false
	}

	return true
}

func NextWay(way Way, offline Offline, isForward bool) (NextWayResult, error) {
	nodes, err := way.Nodes()
	if err != nil {
		return NextWayResult{}, errors.Wrap(err, "could not read way nodes")
	}
	if !way.HasNodes() || nodes.Len() == 0 {
		return NextWayResult{}, nil
	}

	var matchNode Coordinates
	var matchBearingNode Coordinates
	if isForward {
		matchNode = nodes.At(nodes.Len() - 1)
		if nodes.Len() > 1 {
			matchBearingNode = nodes.At(nodes.Len() - 2)
		}
	} else {
		matchNode = nodes.At(0)
		if nodes.Len() > 1 {
			matchBearingNode = nodes.At(1)
		}
	}

	if !PointInBox(matchNode.Latitude(), matchNode.Longitude(), offline.MinLat()-offline.Overlap(), offline.MinLon()-offline.Overlap(), offline.MaxLat()+offline.Overlap(), offline.MaxLon()+offline.Overlap()) {
		return NextWayResult{}, nil
	}

	// Road-axis approach point for the U-turn test; computed once per junction,
	// shared by every candidate check below.
	inAxisPoint := uTurnInAxisPoint(offline, way, isForward)

	matchingWays, err := MatchingWays(way, offline, matchNode)
	if err != nil {
		return NextWayResult{StartPosition: matchNode}, errors.Wrap(err, "could not check for next ways")
	}

	if len(matchingWays) == 0 {
		return NextWayResult{StartPosition: matchNode}, nil
	}

	context := determineRoadContext(way, Position{Latitude: matchNode.Latitude(), Longitude: matchNode.Longitude()})
	if context == CONTEXT_FREEWAY {
		filteredWays := []Way{}
		for _, mWay := range matchingWays {
			name, _ := mWay.Name()
			nameUpper := strings.ToUpper(name)
			if !strings.Contains(nameUpper, "SERVICE") &&
				!strings.Contains(nameUpper, "ACCESS") &&
				!(strings.Contains(nameUpper, "RAMP") && mWay.Lanes() < 2) {
				filteredWays = append(filteredWays, mWay)
			}
		}
		if len(filteredWays) > 0 {
			matchingWays = filteredWays
		}
	}

	curvatureThreshold := 0.15
	if context == CONTEXT_CITY {
		curvatureThreshold = 0.3
	} else if context == CONTEXT_FREEWAY {
		curvatureThreshold = 0.1
	}

	name, _ := way.Name()
	if len(name) > 0 {
		candidates := []Way{}
		for _, mWay := range matchingWays {
			mName, err := mWay.Name()
			if err != nil {
				continue
			}
			if mName == name {
				isForward := NextIsForward(mWay, matchNode)
				if !isForward && mWay.OneWay() {
					continue
				}
				if isUTurn(mWay, matchNode, inAxisPoint) {
					continue
				}

				if nodes.Len() > 1 && isValidConnection(mWay, matchNode, matchBearingNode, curvatureThreshold) {
					candidates = append(candidates, mWay)
				}
			}
		}

		if len(candidates) > 0 {
			bestWay := selectBestCandidate(candidates, matchNode, context)
			isForward := NextIsForward(bestWay, matchNode)
			start, end := GetWayStartEnd(bestWay, isForward)
			return NextWayResult{
				Way:           bestWay,
				StartPosition: start,
				EndPosition:   end,
				IsForward:     isForward,
			}, nil
		}
	}

	ref, _ := way.Ref()
	if len(ref) > 0 {
		candidates := []Way{}
		for _, mWay := range matchingWays {
			mRef, err := mWay.Ref()
			if err != nil {
				continue
			}
			if mRef == ref {
				isForward := NextIsForward(mWay, matchNode)
				if !isForward && mWay.OneWay() {
					continue
				}
				if isUTurn(mWay, matchNode, inAxisPoint) {
					continue
				}

				if nodes.Len() > 1 && isValidConnection(mWay, matchNode, matchBearingNode, curvatureThreshold) {
					candidates = append(candidates, mWay)
				}
			}
		}

		if len(candidates) > 0 {
			bestWay := selectBestCandidate(candidates, matchNode, context)
			isForward := NextIsForward(bestWay, matchNode)
			start, end := GetWayStartEnd(bestWay, isForward)
			return NextWayResult{
				Way:           bestWay,
				StartPosition: start,
				EndPosition:   end,
				IsForward:     isForward,
			}, nil
		}
	}

	if len(ref) > 0 {
		refs := strings.Split(ref, ";")
		candidates := []Way{}
		for _, mWay := range matchingWays {
			mRef, err := mWay.Ref()
			if err != nil {
				continue
			}
			mRefs := strings.Split(mRef, ";")
			hasMatch := false
			for _, r := range refs {
				for _, mr := range mRefs {
					hasMatch = hasMatch || (strings.TrimSpace(r) == strings.TrimSpace(mr))
				}
			}
			if hasMatch {
				isForward := NextIsForward(mWay, matchNode)
				if !isForward && mWay.OneWay() {
					continue
				}
				if isUTurn(mWay, matchNode, inAxisPoint) {
					continue
				}

				if nodes.Len() > 1 && isValidConnection(mWay, matchNode, matchBearingNode, curvatureThreshold) {
					candidates = append(candidates, mWay)
				}
			}
		}

		if len(candidates) > 0 {
			bestWay := selectBestCandidate(candidates, matchNode, context)
			isForward := NextIsForward(bestWay, matchNode)
			start, end := GetWayStartEnd(bestWay, isForward)
			return NextWayResult{
				Way:           bestWay,
				StartPosition: start,
				EndPosition:   end,
				IsForward:     isForward,
			}, nil
		}
	}

	validWays := []Way{}
	for _, mWay := range matchingWays {
		isForward := NextIsForward(mWay, matchNode)
		if !isForward && mWay.OneWay() {
			continue
		}
		if isUTurn(mWay, matchNode, inAxisPoint) {
			continue
		}
		if nodes.Len() > 1 && isValidConnection(mWay, matchNode, matchBearingNode, curvatureThreshold) {
			validWays = append(validWays, mWay)
		}
	}

	if len(validWays) > 0 {
		bestWay := selectBestCandidate(validWays, matchNode, context)
		nextIsForward := NextIsForward(bestWay, matchNode)
		start, end := GetWayStartEnd(bestWay, nextIsForward)
		return NextWayResult{
			Way:           bestWay,
			StartPosition: start,
			EndPosition:   end,
			IsForward:     nextIsForward,
		}, nil
	}

	// Last-resort fallback still must not chain onto a direction reversal: a
	// missing next way is safer than a phantom target on the opposite carriageway.
	for _, mWay := range matchingWays {
		if isUTurn(mWay, matchNode, inAxisPoint) {
			continue
		}
		nextIsForward := NextIsForward(mWay, matchNode)
		start, end := GetWayStartEnd(mWay, nextIsForward)
		return NextWayResult{
			Way:           mWay,
			StartPosition: start,
			EndPosition:   end,
			IsForward:     nextIsForward,
		}, nil
	}

	return NextWayResult{StartPosition: matchNode}, nil
}

// axisEndpoint walks the node list from startIdx in direction step (+1/-1),
// accumulating along-road distance until at least minDist meters, and returns
// the node reached plus the distance actually accumulated (clamped at the way
// end when the way is shorter than minDist).
func axisEndpoint(nodes capnp.StructList[Coordinates], startIdx int, step int, minDist float64) (Coordinates, float64) {
	point := nodes.At(startIdx)
	dist := 0.0
	for i := startIdx + step; i >= 0 && i < nodes.Len(); i += step {
		next := nodes.At(i)
		dist += DistanceToPoint(point.Latitude()*TO_RADIANS, point.Longitude()*TO_RADIANS, next.Latitude()*TO_RADIANS, next.Longitude()*TO_RADIANS)
		point = next
		if dist >= minDist {
			break
		}
	}
	return point, dist
}

// uTurnInAxisPoint returns the road-axis reference point that lies at least
// U_TURN_AXIS_DIST meters BEHIND the junction node, found by walking the
// from-way backward along its traversal direction. When the from-way itself is
// shorter than the axis distance (dual carriageways are chopped into tiny ways
// right at crossover junctions, e.g. the 10.8 m crest stub of 31st Ave N), the
// walk continues into the unique connected same-name way at the reached end,
// up to U_TURN_AXIS_MAX_HOPS extra ways; ambiguity (0 or >1 same-name
// continuations) or a missing name clamps the walk conservatively where it is.
// The returned point may equal the junction node for degenerate ways; callers
// must treat that as "no bearing context".
func uTurnInAxisPoint(offline Offline, fromWay Way, fromForward bool) Coordinates {
	nodes, err := fromWay.Nodes()
	if err != nil || nodes.Len() < 2 {
		return Coordinates{}
	}
	startIdx, step := nodes.Len()-1, -1
	if !fromForward {
		startIdx, step = 0, 1
	}
	point, dist := axisEndpoint(nodes, startIdx, step, U_TURN_AXIS_DIST)

	name, _ := fromWay.Name()
	cur := fromWay
	for hops := 0; dist < U_TURN_AXIS_DIST && hops < U_TURN_AXIS_MAX_HOPS; hops++ {
		if len(name) == 0 {
			break
		}
		matching, err := MatchingWays(cur, offline, point)
		if err != nil {
			break
		}
		var next Way
		sameName := 0
		for _, m := range matching {
			mName, err := m.Name()
			if err != nil || mName != name {
				continue
			}
			sameName++
			next = m
		}
		if sameName != 1 {
			break
		}
		nNodes, err := next.Nodes()
		if err != nil || nNodes.Len() < 2 {
			break
		}
		sIdx, sStep := nNodes.Len()-1, -1
		first := nNodes.At(0)
		if first.Latitude() == point.Latitude() && first.Longitude() == point.Longitude() {
			sIdx, sStep = 0, 1
		}
		var d float64
		point, d = axisEndpoint(nNodes, sIdx, sStep, U_TURN_AXIS_DIST-dist)
		dist += d
		cur = next
	}
	return point
}

// isUTurn reports whether continuing through the junction at matchNode onto
// the candidate reverses the road's axis by more than 150° (see
// U_TURN_BEARING_DELTA_COS). Both sides are measured as ROAD-AXIS chord
// bearings over at least U_TURN_AXIS_DIST meters of along-road distance:
// inAxisPoint (see uTurnInAxisPoint) -> matchNode for the approach, and
// matchNode -> a point walked U_TURN_AXIS_DIST along the candidate in its
// traversal direction (per NextIsForward, clamped at the candidate's end) for
// the continuation. Single adjacent-segment bearings are NOT sufficient: at
// divided-carriageway crossover junctions the ~9 m median jog segments rotate
// toward each other and shave a true 180° reversal down to ~117-127°. The
// junction-curvature check in isValidConnection cannot catch reversals
// either: with ~100 m legs and a few meters of lateral offset the
// circumcircle through the three junction nodes of a hairpin is nearly flat.
func isUTurn(candidate Way, matchNode, inAxisPoint Coordinates) bool {
	// No bearing context (degenerate from-way) -> cannot judge, do not reject.
	if !capnp.Struct(inAxisPoint).IsValid() || !capnp.Struct(matchNode).IsValid() {
		return false
	}
	if inAxisPoint.Latitude() == matchNode.Latitude() && inAxisPoint.Longitude() == matchNode.Longitude() {
		return false
	}
	nodes, err := candidate.Nodes()
	if err != nil || nodes.Len() < 2 {
		return false
	}

	// Walk the candidate away from matchNode in the direction the chain would
	// actually drive it (see NextIsForward).
	startIdx, step := 0, 1
	if !NextIsForward(candidate, matchNode) {
		startIdx, step = nodes.Len()-1, -1
	}
	outPoint, _ := axisEndpoint(nodes, startIdx, step, U_TURN_AXIS_DIST)
	if outPoint.Latitude() == matchNode.Latitude() && outPoint.Longitude() == matchNode.Longitude() {
		return false
	}

	inBearing := Bearing(inAxisPoint.Latitude(), inAxisPoint.Longitude(), matchNode.Latitude(), matchNode.Longitude())
	outBearing := Bearing(matchNode.Latitude(), matchNode.Longitude(), outPoint.Latitude(), outPoint.Longitude())
	return math.Cos(outBearing-inBearing) < U_TURN_BEARING_DELTA_COS
}

func isValidConnection(way Way, matchNode, bearingNode Coordinates, maxCurvature float64) bool {
	nodes, err := way.Nodes()
	if err != nil || nodes.Len() < 2 {
		return false
	}

	var nextBearingNode Coordinates
	if matchNode.Latitude() == nodes.At(0).Latitude() && matchNode.Longitude() == nodes.At(0).Longitude() {
		nextBearingNode = nodes.At(1)
	} else {
		nextBearingNode = nodes.At(nodes.Len() - 2)
	}

	curv, _, _ := GetCurvature(bearingNode.Latitude(), bearingNode.Longitude(), matchNode.Latitude(), matchNode.Longitude(), nextBearingNode.Latitude(), nextBearingNode.Longitude())
	return math.Abs(curv) <= maxCurvature
}

func selectBestCandidate(candidates []Way, matchNode Coordinates, context RoadContext) Way {
	if len(candidates) == 1 {
		return candidates[0]
	}

	bestWay := candidates[0]
	bestScore := float64(-1000)

	for _, way := range candidates {
		score := float64(getRoadPriority(way, context))

		lanes := way.Lanes()
		if lanes > 0 {
			laneWeight := 2.0
			if context == CONTEXT_FREEWAY {
				laneWeight = 4.0
			} else if context == CONTEXT_CITY {
				laneWeight = 1.0
			}
			score += float64(lanes) * laneWeight
		}

		if score > bestScore {
			bestScore = score
			bestWay = way
		}
	}

	return bestWay
}

func DistanceToEndOfWay(pos Position, way Way, isForward bool) (float64, error) {
	distanceResult, err := DistanceToWay(pos, way)
	if err != nil {
		return 0, err
	}
	lat := distanceResult.LineEnd.Latitude()
	lon := distanceResult.LineEnd.Longitude()
	dist := DistanceToPoint(pos.Latitude*TO_RADIANS, pos.Longitude*TO_RADIANS, lat*TO_RADIANS, lon*TO_RADIANS)
	stopFiltering := false
	nodes, err := way.Nodes()
	if err != nil {
		return 0, err
	}
	for i := 0; i < nodes.Len(); i++ {
		index := i
		if !isForward {
			index = nodes.Len() - 1 - i
		}
		node := nodes.At(index)
		nLat := node.Latitude()
		nLon := node.Longitude()
		if node.Latitude() == lat && node.Longitude() == lon && !stopFiltering {
			stopFiltering = true
		}
		if !stopFiltering {
			continue
		}
		dist += DistanceToPoint(lat*TO_RADIANS, lon*TO_RADIANS, nLat*TO_RADIANS, nLon*TO_RADIANS)
		lat = nLat
		lon = nLon
	}
	return dist, nil
}

func NextWays(pos Position, currentWay CurrentWay, offline Offline, isForward bool) ([]NextWayResult, error) {
	nextWays := []NextWayResult{}
	dist := 0.0
	wayIdx := currentWay.Way
	forward := isForward
	startPos := pos
	for dist < float64(MIN_WAY_DIST) {
		d, err := DistanceToEndOfWay(startPos, wayIdx, forward)
		if err != nil || d <= 0 {
			break
		}
		dist += d
		nw, err := NextWay(wayIdx, offline, forward)
		if err != nil {
			break
		}
		nextWays = append(nextWays, nw)
		wayIdx = nw.Way
		startPos = Position{
			Latitude:  nw.StartPosition.Latitude(),
			Longitude: nw.StartPosition.Longitude(),
		}
		forward = nw.IsForward
	}

	if len(nextWays) == 0 {
		nextWay, err := NextWay(currentWay.Way, offline, isForward)
		if err != nil {
			return []NextWayResult{}, err
		}
		nextWays = append(nextWays, nextWay)
	}

	return nextWays, nil
}
