package main

import (
	"math"

	"capnproto.org/go/capnp/v3"
	"github.com/pkg/errors"
)

var (
	R                = 6373000.0           // approximate radius of earth in meters
	LANE_WIDTH       = 3.7                 // meters
	QUERY_RADIUS     = float64(3000)       // meters
	PADDING          = 10 / R * TO_DEGREES // 10 meters in degrees
	TO_RADIANS       = math.Pi / 180
	TO_DEGREES       = 180 / math.Pi
	TARGET_LAT_ACCEL = 2.0 // m/s^2
)

func Dot(ax float64, ay float64, bx float64, by float64) float64 {
	return (ax * bx) + (ay * by)
}

func PointOnLine(startLat float64, startLon float64, endLat float64, endLon float64, lat float64, lon float64) (float64, float64) {
	aplat := lat - startLat
	aplon := lon - startLon

	ablat := endLat - startLat
	ablon := endLon - startLon

	t := Dot(aplat, aplon, ablat, ablon) / Dot(ablat, ablon, ablat, ablon)

	if t < 0 {
		t = 0
	}
	if t > 1 {
		t = 1
	}

	latitude := startLat + t*ablat
	longitude := startLon + t*ablon

	return latitude, longitude
}

// arguments should be in radians
func DistanceToPoint(ax float64, ay float64, bx float64, by float64) float64 {
	a := math.Sin((bx-ax)/2)*math.Sin((bx-ax)/2) + math.Cos(ax)*math.Cos(bx)*math.Sin((by-ay)/2)*math.Sin((by-ay)/2)
	c := 2 * math.Atan2(math.Sqrt(a), math.Sqrt(1-a))

	return R * c // in metres
}

func Vector(latA float64, lonA float64, latB float64, lonB float64) (float64, float64) {
	dlon := lonB - lonA
	x := math.Sin(dlon) * math.Cos(latB)
	y := math.Cos(latA)*math.Sin(latB) - (math.Sin(latA) * math.Cos(latB) * math.Cos(dlon))
	return x, y
}

func Bearing(latA float64, lonA float64, latB float64, lonB float64) float64 {
	latA = latA * TO_RADIANS
	latB = latB * TO_RADIANS
	lonA = lonA * TO_RADIANS
	lonB = lonB * TO_RADIANS
	x, y := Vector(latA, lonA, latB, lonB)
	return math.Atan2(x, y)
}

type Curvature struct {
	Latitude  float64 `json:"latitude"`
	Longitude float64 `json:"longitude"`
	Curvature float64 `json:"curvature"`
	// BluePilot: the point sits on an interchange connector (see latBudget);
	// internal context only, never serialized
	IsRamp bool `json:"-"`
	// End BluePilot
}

// Median-crossover signature: a way boundary that switches between two-way and
// oneway AND whose boundary-adjacent traversal segments are both shorter than
// this (meters) is the node-split jog where a divided road's carriageways meet
// (the short jog segments rotate toward each other and fake a sharp bend that
// no traffic ever drives). Real curves have longer approach segments.
const CROSSOVER_MAX_BOUNDARY_SEGMENT = 15.0

// Curvature written into suppressed jog indices; same "basically straight"
// value the merge/split flattening uses.
const FLATTENED_CURVATURE = 0.0015

// Measurability gates on the raw per-triple curvature before it is allowed to
// override the smoothed value (see GetStateCurvatures). A three-node
// circumcircle only resolves a radius if the arc it spans is long enough to
// carry signal and the sagitta it implies, arc^2/(8R), rises clear of OSM
// digitisation error. Below either bound the raw sample is measuring node
// noise: a straight arterial digitised with a 3.4 m node pair implied
// R = 39.7 m, and interstate corridors at ~12 m spacing implied curves that
// are not there. The smoothed value spans roughly three times the arc and
// stays reliable at those scales, so it is what gets published instead.
const MIN_PEAK_ARC = 25.0    // meters of arc across the triple
const MIN_PEAK_SAGITTA = 1.5 // meters of implied deviation from a straight chord

// The raw term also only rescues genuinely TIGHT bends (radius at or below
// 1/this, i.e. 120 m). A polyline is a coarse sampling of a smooth road, so on
// gentle geometry the total turn lands unevenly on the nodes and whichever
// node caught the largest share reads as a local corner: on I 40 a real
// 569-851 m corridor turned 11.2 deg at one node and the triple there honestly
// measures R = 227 m. Averaging is the right answer for that, and for anything
// else at highway scale. Below 120 m the sampling argument no longer applies —
// a bend that tight is a real feature of the road, it is precisely what the
// average smears into its straighter neighbours, and it is the only case the
// peak term exists to rescue.
const MIN_PEAK_CURVATURE = 1.0 / 120.0 // 1/m

// BluePilot: human-referenced lateral budgets.
//
// TARGET_LAT_ACCEL is no longer a flat budget: it is the budget AT THE 30 MPH
// ANCHOR (13.4 m/s), and the budget at other speeds follows the AASHTO Green
// Book side-friction comfort curve, normalised to that anchor. Verified OSM
// trace bands over Nashville put the median human on a 30 mph arterial curve
// at 2.0 m/s^2 measured against the MAP radius (p25 1.7, p75 2.3) - exactly
// the AASHTO fmax value at 30 mph (0.20 g). Below the anchor the shape
// follows AASHTO up toward 0.32 g. ABOVE the anchor the measured drivers are
// far bolder than AASHTO's design values: on a 62-pass interstate sweeper
// they hold 72 mph at 0.6-0.8 m/s^2 without lifting, on an R~490 m curve
// they hold 63 mph at 1.64 m/s^2, and they only start braking around
// 2.1 m/s^2 (65 -> 55 mph on an R~285 m curve) - where AASHTO's 0.12 g would
// have them lift a full 10 mph earlier. So the high-speed tail flattens at
// 0.16 g (0.8 x anchor) instead of falling to 0.08: gentle sweepers stay
// unbound exactly as humans leave them, and genuinely tight high-speed
// curves still bind near the measured human speed. Interpolated linearly,
// clamped at the table ends.
//
// Ramps are their own regime, not a point on that curve: on the verified
// I-65 -> Old Hickory loop (map R 69 m) the median human runs 3.4 m/s^2, and
// on the I-440 exit ramp 3.1-3.9 m/s^2 at 55 mph - drivers accept roughly
// 0.25-0.3 g on connectors they chose to take (SHRP2 ramp studies agree),
// nearly twice their open-road budget at the same speed. A way is a ramp when
// it is oneway with neither name nor ref, the interchange-connector signature
// in this tile schema (both verified ramps match it; named city one-ways do
// not). The ramp budget is flat across speed (measured flat from the 34 mph
// loop to the 55 mph diagonal exit), scaled by the profile anchor and capped
// at RAMP_LAT_CAP - just above ISO 11270's 3.0 m/s^2 assisted-steering bound
// and below the 3.9 m/s^2 hard ceiling naturalistic studies see anywhere.
var COMFORT_SPEED_MS = []float64{6.7, 8.9, 11.2, 13.4, 15.6, 17.9, 20.1, 24.6, 29.1, 35.8}
var COMFORT_FMAX_G = []float64{0.32, 0.27, 0.23, 0.20, 0.19, 0.18, 0.17, 0.165, 0.16, 0.16}

const COMFORT_ANCHOR_G = 0.20 // fmax at the 30 mph anchor; the shape divides by this
const RAMP_LAT_MULT = 1.55    // ramp budget = anchor budget x this ...
const RAMP_LAT_CAP = 3.4      // ... capped here (m/s^2)

// comfortShape returns fmax(v)/fmax(30 mph): 1.0 at the anchor, ~1.6 at
// parking-lot speed, flat at 0.8 from ~65 mph up.
func comfortShape(v float64) float64 {
	t := COMFORT_SPEED_MS
	if v <= t[0] {
		return COMFORT_FMAX_G[0] / COMFORT_ANCHOR_G
	}
	if v >= t[len(t)-1] {
		return COMFORT_FMAX_G[len(t)-1] / COMFORT_ANCHOR_G
	}
	for i := 1; i < len(t); i++ {
		if v <= t[i] {
			f := (v - t[i-1]) / (t[i] - t[i-1])
			g := COMFORT_FMAX_G[i-1] + f*(COMFORT_FMAX_G[i]-COMFORT_FMAX_G[i-1])
			return g / COMFORT_ANCHOR_G
		}
	}
	return COMFORT_FMAX_G[len(t)-1] / COMFORT_ANCHOR_G
}

// latBudget is the lateral acceleration allowed at speed v on this kind of
// way. TARGET_LAT_ACCEL is the profile's 30 mph anchor (MapTargetLatA).
func latBudget(v float64, isRamp bool) float64 {
	if isRamp {
		return math.Min(TARGET_LAT_ACCEL*RAMP_LAT_MULT, RAMP_LAT_CAP)
	}
	return TARGET_LAT_ACCEL * comfortShape(v)
}

// BluePilot: loop ramps are DESIGNED as circular arcs, and their whole-way
// geometry says so more reliably than any local estimate: total sweep over
// total length gives R 74 m on the verified I-65 -> Old Hickory loop (driven
// 62-68 m, tile noded so coarsely its raw triples claim 42) and R 56 m on the
// Briley loop (whose 6-9 m node legs make raw triples claim 23). For a ramp
// way that sweeps most of a circle, published curvature on its points is
// clamped into a band around that mean: capped so digitising noise cannot
// fake a much tighter coil, and floored - only where the local estimate
// already shows real curvature, so a straight lead-in stays straight - so
// under-reading cannot let the car carry mainline speed into the coil.
const LOOP_MIN_SWEEP_DEG = 170.0 // a ramp turning at least this much is a loop
const LOOP_CURV_CAP = 1.25       // x mean sweep curvature
const LOOP_CURV_FLOOR = 0.85     // x mean sweep curvature ...
const LOOP_FLOOR_GATE = 0.5      // ... applied only where local curv is already this fraction of mean

// lessNaNFirst is the ordering sort.Float64s applies: NaN sorts before every
// number (sort.Float64s is slices.Sort, which compares with cmp.Less).
func lessNaNFirst(x, y float64) bool {
	return x < y || (math.IsNaN(x) && !math.IsNaN(y))
}

// median3 is the middle of three values under that same ordering - a 3-element
// sorting network in place of allocating a slice header and calling into
// slices.Sort, which costs ~12 ns per gated sample and is the only new
// per-sample cost that scales with chain length (2.3x faster on a 151-sample
// interstate chain).
//
// The NaN handling is load-bearing rather than defensive: GetCurvature returns
// NaN whenever a degenerate node triple drives its circumcircle area imaginary,
// and the dense gate above cannot reject it because every comparison against
// NaN is false. Matching sort's NaN-first ordering is what keeps this
// bit-identical on those triples.
func median3(a, b, c float64) float64 {
	if lessNaNFirst(b, a) {
		a, b = b, a
	}
	if lessNaNFirst(c, b) {
		b = c
	}
	if lessNaNFirst(b, a) {
		b = a
	}
	return b
}

// loopMeanCurvature returns the coil's sweep/length curvature when the way is
// a loop ramp, else 0. isRamp is passed in rather than recomputed: both call
// sites need isRampWay(way) for the per-point ramp budget anyway, and this
// function is the only other caller, so computing it once halves the
// isRampWay traffic (9.4 -> 4.7 calls per tick on the recorded corpus).
// The mean is taken over the way's curved CORE: loop
// ways often carry a straight lead-in/out (the verified Old Hickory loop
// spends its first 66 m and last ~60 m nearly straight), and including those
// dilutes the mean from the ~70 m the coil actually is to over 100 m. Ends
// are trimmed while their local turn rate is under half the whole-way mean.
func loopMeanCurvature(way Way, isRamp bool) float64 {
	if !isRamp {
		return 0
	}
	nodes, err := way.Nodes()
	if err != nil || nodes.Len() < 4 {
		return 0
	}
	n := nodes.Len()
	segLen := make([]float64, n) // segLen[i]: length of segment i-1 -> i
	turn := make([]float64, n)   // turn[i]: |heading change| at interior node i
	prevBearing := 0.0
	length := 0.0
	sweep := 0.0
	for i := 1; i < n; i++ {
		a, b := nodes.At(i-1), nodes.At(i)
		segLen[i] = DistanceToPoint(a.Latitude()*TO_RADIANS, a.Longitude()*TO_RADIANS,
			b.Latitude()*TO_RADIANS, b.Longitude()*TO_RADIANS)
		length += segLen[i]
		bearing := Bearing(a.Latitude(), a.Longitude(), b.Latitude(), b.Longitude())
		if i > 1 {
			d := bearing - prevBearing
			for d > math.Pi {
				d -= 2 * math.Pi
			}
			for d < -math.Pi {
				d += 2 * math.Pi
			}
			turn[i-1] = math.Abs(d)
			sweep += turn[i-1]
		}
		prevBearing = bearing
	}
	if length <= 0 || sweep < LOOP_MIN_SWEEP_DEG*TO_RADIANS {
		return 0
	}
	wayMean := sweep / length

	// trim straight ends: drop interior nodes whose local curvature (turn over
	// the node-centred arc) is under half the whole-way mean
	lo, hi := 1, n-2 // interior node range
	localCurv := func(j int) float64 {
		arc := (segLen[j] + segLen[j+1]) / 2
		if arc <= 0 {
			return 0
		}
		return turn[j] / arc
	}
	for lo < hi && localCurv(lo) < 0.5*wayMean {
		lo++
	}
	for hi > lo && localCurv(hi) < 0.5*wayMean {
		hi--
	}
	coreSweep := 0.0
	coreLen := 0.0
	for j := lo; j <= hi; j++ {
		coreSweep += turn[j]
		coreLen += (segLen[j] + segLen[j+1]) / 2
	}
	if coreLen <= 0 || coreSweep <= 0 {
		return wayMean
	}
	return coreSweep / coreLen
}

// isRampWay: interchange-connector signature (see the budget comment above).
//
// The tag tests are pointer-presence checks, not text reads: Name() resolves a
// far pointer and copies the bytes into a fresh string (129 ns and a 24 B
// allocation on a named way), while HasName() is a raw pointer-word compare
// that never touches the text. capnp's SetText writes a NULL pointer for the
// empty string (struct.go: `if v == "" { return p.SetPtr(i, Ptr{}) }`), so an
// absent pointer is exactly an absent tag; verified over all 11037 ways of the
// shipped Nashville tile with zero divergences, and pinned by
// TestHasNameMatchesEmptyText so a future tile generator cannot break it
// silently. Were a tile ever to store a zero-length text rather than a null
// pointer, this returns false where the old form returned true - the way loses
// its ramp budget and falls back to the lower open-road one, which is the
// conservative direction.
func isRampWay(way Way) bool {
	return way.OneWay() && !way.HasName() && !way.HasRef()
}

// BluePilot: dense-noding curvature cap - what the road sustains, not what
// one noisy triple claims.
//
// A three-node circumcircle reads tracing noise on densely-noded polylines:
// the verified Old Hickory loop is a steady 62-68 m circle driven and mapped,
// yet its raw triples alternate 42 / 174 / 76 m because ~1 m of digitising
// wiggle on 13-17 m node spacing is a large local angle. The median of the
// five triples around the anchor is immune to that alternation while still
// reporting a genuinely sustained bend at full strength, so wherever the
// noding is dense (every one of those five triples spans no more than
// DENSE_TRIPLE_ARC of road) the published curvature is capped at
// DENSE_CAP_FACTOR x that median. Sparse polylines - the under-noded bends
// the peak-preserving max exists to rescue, noded 70 m apart - never satisfy
// the arc gate and are left alone.
const DENSE_TRIPLE_ARC = 45.0 // meters; all five triples must be at most this long
const DENSE_CAP_FACTOR = 1.2  // slack over the median for short real features
// End BluePilot

// boundarySegmentLength returns the length (meters) of the way's traversal
// segment adjacent to a chain boundary: the segment the chain leaves the way
// on (entering=false) or enters it on (entering=true), given the way's
// traversal direction. Returns +Inf when the way is degenerate so callers
// never classify a boundary from missing data.
func boundarySegmentLength(way Way, forward bool, entering bool) float64 {
	nodes, err := way.Nodes()
	if err != nil || nodes.Len() < 2 {
		return math.Inf(1)
	}
	var a, b Coordinates
	if forward == entering {
		// Entering forward or leaving backward: segment at node-order start.
		a, b = nodes.At(0), nodes.At(1)
	} else {
		// Leaving forward or entering backward: segment at node-order end.
		a, b = nodes.At(nodes.Len()-2), nodes.At(nodes.Len()-1)
	}
	return DistanceToPoint(a.Latitude()*TO_RADIANS, a.Longitude()*TO_RADIANS, b.Latitude()*TO_RADIANS, b.Longitude()*TO_RADIANS)
}

func GetStateCurvatures(state *State) ([]Curvature, error) {
	nodes, err := state.CurrentWay.Way.Nodes()
	if err != nil {
		return []Curvature{}, errors.Wrap(err, "could not read way nodes")
	}
	num_points := nodes.Len()
	all_nodes := []capnp.StructList[Coordinates]{nodes}
	all_nodes_direction := []bool{state.CurrentWay.OnWay.IsForward}
	all_nodes_is_merge_or_split := []bool{false}
	all_nodes_is_crossover := []bool{false}
	// BluePilot: which chain entries are interchange connectors, for the
	// per-point ramp lateral budget (see latBudget), and the loop-arc mean
	// curvature for entries that are loop ramps (see loopMeanCurvature)
	currentIsRamp := isRampWay(state.CurrentWay.Way)
	all_nodes_is_ramp := []bool{currentIsRamp}
	all_nodes_loop_curv := []float64{loopMeanCurvature(state.CurrentWay.Way, currentIsRamp)}
	// End BluePilot
	lastWay := state.CurrentWay.Way
	for _, nextWay := range state.NextWays {
		nwNodes, err := nextWay.Way.Nodes()
		if err != nil {
			continue
		}
		if nwNodes.Len() > 0 {
			num_points += nwNodes.Len() - 1
		}
		lastForward := all_nodes_direction[len(all_nodes_direction)-1]
		all_nodes = append(all_nodes, nwNodes)
		all_nodes_direction = append(all_nodes_direction, nextWay.IsForward)
		// Only treat a lane-count change as a merge/split when BOTH ways have a
		// known (tagged, > 0) lane count. Lanes() == 0 means "untagged", and
		// comparing against it flattened REAL curve targets at every boundary
		// between a tagged and an untagged way.
		lanesKnown := lastWay.Lanes() > 0 && nextWay.Way.Lanes() > 0
		isMergeOrSplit := lanesKnown &&
			(lastWay.Lanes() < nextWay.Way.Lanes() ||
				(lastWay.Lanes() > nextWay.Way.Lanes() && !lastWay.OneWay() && nextWay.Way.OneWay()))
		all_nodes_is_merge_or_split = append(all_nodes_is_merge_or_split, isMergeOrSplit)
		// Median-crossover signature (see CROSSOVER_MAX_BOUNDARY_SEGMENT): a
		// two-way <-> oneway transition with very short boundary-adjacent
		// segments on both sides. Only the curvature indices at/immediately
		// adjacent to the boundary node get flattened — NOT a 15 m sweep, which
		// would re-mask real curves that start right after the jog (the 31st Ave
		// N crest curve peaks ~14 m from the crossover node).
		isCrossover := lastWay.OneWay() != nextWay.Way.OneWay() &&
			boundarySegmentLength(lastWay, lastForward, false) < CROSSOVER_MAX_BOUNDARY_SEGMENT &&
			boundarySegmentLength(nextWay.Way, nextWay.IsForward, true) < CROSSOVER_MAX_BOUNDARY_SEGMENT
		all_nodes_is_crossover = append(all_nodes_is_crossover, isCrossover)
		// BluePilot
		nextIsRamp := isRampWay(nextWay.Way)
		all_nodes_is_ramp = append(all_nodes_is_ramp, nextIsRamp)
		all_nodes_loop_curv = append(all_nodes_loop_curv, loopMeanCurvature(nextWay.Way, nextIsRamp))
		// End BluePilot
		lastWay = nextWay.Way
	}

	x_points := make([]float64, num_points)
	y_points := make([]float64, num_points)
	// BluePilot: per-point ramp flag, taken from the chain entry each point is
	// copied out of (a shared boundary node takes the entry that owns it in
	// the concatenation; one node of slack either way does not matter to a
	// budget that changes by way, not by node)
	point_is_ramp := make([]bool, num_points)
	point_loop_curv := make([]float64, num_points)
	// End BluePilot

	merge_or_split_nodes := []int{}
	crossover_nodes := []int{}
	all_nodes_idx := 0
	nodes_idx := 0
	for i := 0; i < num_points; i++ {
		var index int
		forward := all_nodes_direction[all_nodes_idx]
		if forward {
			index = nodes_idx
			if all_nodes_idx > 0 {
				index += 1
			}
		} else {
			index = all_nodes[all_nodes_idx].Len() - nodes_idx - 1
			if all_nodes_idx > 0 {
				index -= 1
			}
		}
		node := all_nodes[all_nodes_idx].At(index)
		x_points[i] = node.Latitude()
		y_points[i] = node.Longitude()
		// BluePilot
		point_is_ramp[i] = all_nodes_is_ramp[all_nodes_idx]
		point_loop_curv[i] = all_nodes_loop_curv[all_nodes_idx]
		// End BluePilot

		nodes_idx += 1
		if nodes_idx == all_nodes[all_nodes_idx].Len() || (nodes_idx == all_nodes[all_nodes_idx].Len()-1 && all_nodes_idx > 0) {
			all_nodes_idx += 1
			nodes_idx = 0
			if all_nodes_idx < len(all_nodes_is_merge_or_split) && all_nodes_is_merge_or_split[all_nodes_idx] {
				merge_or_split_nodes = append(merge_or_split_nodes, i)
			}
			if all_nodes_idx < len(all_nodes_is_crossover) && all_nodes_is_crossover[all_nodes_idx] {
				crossover_nodes = append(crossover_nodes, i)
			}
		}
	}

	curvatures, arc_lengths, err := GetCurvatures(x_points, y_points)
	if err != nil {
		return []Curvature{}, errors.Wrap(err, "could not get curvatures from points")
	}

	// BluePilot: per-triple dense-noding cap (see DENSE_TRIPLE_ARC). For each
	// curvature sample k, the cap is DENSE_CAP_FACTOR x median of the raw
	// triples k-1..k+1, valid only when all three exist and all are short.
	dense_cap := make([]float64, len(curvatures))
	for k := range dense_cap {
		dense_cap[k] = math.Inf(1)
		if k < 1 || k+1 >= len(curvatures) {
			continue
		}
		dense := true
		var three [3]float64
		for m := 0; m < 3; m++ {
			// A triple the merge/split or crossover passes flattened is not
			// evidence about the road: the jogs those passes suppress are
			// exactly the short segments that would fake "dense noding" here,
			// and their flattened values would drag the median under a real
			// curve sitting right next to the jog (the 31st Ave N crest).
			if arc_lengths[k-1+m] > DENSE_TRIPLE_ARC || curvatures[k-1+m] == FLATTENED_CURVATURE {
				dense = false
				break
			}
			three[m] = curvatures[k-1+m]
		}
		if !dense {
			continue
		}
		dense_cap[k] = DENSE_CAP_FACTOR * median3(three[0], three[1], three[2])
	}
	// End BluePilot

	// set the merge nodes to be straight to help balance out issues with map representation
	for _, merge_or_split_node := range merge_or_split_nodes {
		if merge_or_split_node >= 2 {
			curvatures[merge_or_split_node-2] = 0.0015
			curvatures[merge_or_split_node-1] = 0.0015
		}
		// also include nodes within 15 meters
		for i := merge_or_split_node - 3; i >= 0; i-- {
			if DistanceToPoint(x_points[merge_or_split_node]*TO_RADIANS, y_points[merge_or_split_node]*TO_RADIANS, x_points[i]*TO_RADIANS, y_points[i]*TO_RADIANS) > 15 {
				break
			}
			curvatures[i] = 0.0015
		}
		// also include forward nodes within 15 meters
		for i := merge_or_split_node; i < len(curvatures); i++ {
			if DistanceToPoint(x_points[merge_or_split_node]*TO_RADIANS, y_points[merge_or_split_node]*TO_RADIANS, x_points[i]*TO_RADIANS, y_points[i]*TO_RADIANS) > 15 {
				break
			}
			curvatures[i] = 0.0015
		}
	}

	// Suppress the median-crossover jog artifact: flatten ONLY the curvature
	// samples centered on the boundary node and the node immediately before it
	// (curvatures[k] is centered on point k+1, so centers b-1 and b are
	// indices b-2 and b-1). Those two triples have BOTH legs inside the
	// crossover jog and carry the phantom bend. Deliberately narrower than the
	// merge/split 15 m sweep above — the sample centered one node past the
	// boundary already overlaps the real curve the crossover sits on (the 31st
	// Ave N crest curve peaks ~14 m from the crossover node) and must stay
	// visible in the published targets.
	for _, b := range crossover_nodes {
		for k := b - 2; k <= b-1; k++ {
			if k >= 0 && k < len(curvatures) {
				curvatures[k] = FLATTENED_CURVATURE
			}
		}
	}

	average_curvatures, err := GetAverageCurvatures(curvatures, arc_lengths)
	if err != nil {
		return []Curvature{}, errors.Wrap(err, "could not get average curvatures from curvatures")
	}

	// BluePilot: peak-preserving curvature. The 3-sample arc-length-weighted
	// average smears a curvature peak into its straight neighbours: at the 31st
	// Ave N pre-light bend the raw circumcircle triple gives R = 66.2 m, but
	// neighbours of R = 318.1 m and R = 2543.8 m with comparable arc weights
	// (76.7 / 69.3 / 68.6 m) dilute the published value to R = 166.0 m — an
	// 18.2 m/s (40.8 mph) target for a bend driven at 54-67 m radius and
	// 2.3 m/s^2 lateral. Publish the elementwise maximum of the averaged value
	// and the RAW curvature of the triple centred on the SAME node, so the
	// average can only ever soften the approach, never erase the peak
	// (R_out = min(R_avg, R_raw_center)).
	//
	// Index mapping: output i is anchored at x_points[i+2]; curvatures[k] is
	// centred on x_points[k+1], so the raw sample sharing output i's anchor is
	// curvatures[i+1].
	//
	// This runs AFTER the merge/split and crossover writes into curvatures[]
	// above, so a sample those passes deliberately flattened stays flattened —
	// the max can never resurrect a suppressed artifact.
	// The raw triple is only allowed to win where it can actually resolve the
	// radius it claims. A three-node circumcircle measures curvature through
	// the sagitta it implies, arc^2/(8R); once that drops toward OSM's own
	// digitisation error the raw value is reading node noise, not road. On a
	// straight stretch of Hillsboro Pike a 3.4 m node pair implied R = 39.7 m
	// (0.15 m of sagitta) and turned a 45 mph road into an 18 mph target,
	// and interstate corridors digitised at ~12 m spacing invented curves
	// worth 15-25 mph. The averaged value spans ~3x the arc and stays
	// trustworthy at those scales, so where the raw sample is unresolvable we
	// simply keep the average. The bend this whole change exists for clears
	// both gates comfortably: 67.6 m of arc and 8.6 m of sagitta.
	published_curvatures := make([]float64, len(average_curvatures))
	for i := range average_curvatures {
		published_curvatures[i] = average_curvatures[i]

		raw := curvatures[i+1]
		arc := arc_lengths[i+1]
		if raw >= MIN_PEAK_CURVATURE && arc >= MIN_PEAK_ARC && arc*arc*raw/8 >= MIN_PEAK_SAGITTA {
			published_curvatures[i] = math.Max(published_curvatures[i], raw)
		}

		// BluePilot: where the noding is dense enough for the three-triple
		// median to be trusted, the preserved PEAK may not claim the road
		// turns much harder than its neighbourhood sustains - but the cap is
		// floored at the arc-weighted average, because the average is already
		// the anti-noise estimate and near suppressed jogs the raw
		// neighbourhood under-reads (the sharp part of the 31st Ave N crest
		// lives in triples the crossover pass deliberately flattened). Net
		// effect: the cap can only strip the raw-peak excess, never cut into
		// what the averaging itself published. This is what stops a 65 m loop
		// mapped with 13-17 m nodes from publishing a 42 m phantom (a 24 mph
		// target on a loop humans drive at 34), while sparse under-noded bends
		// and jog-adjacent curves keep their published values.
		if cap := math.Max(average_curvatures[i], dense_cap[i+1]); published_curvatures[i] > cap {
			published_curvatures[i] = cap
		}

		// Loop-arc clamp (see loopMeanCurvature): points on a loop ramp are
		// banded around the way's mean sweep curvature. Applied last: the
		// design geometry of the coil outranks every local estimate on it.
		if mean := point_loop_curv[i+2]; mean > 0 {
			if hi := LOOP_CURV_CAP * mean; published_curvatures[i] > hi {
				published_curvatures[i] = hi
			}
			if lo := LOOP_CURV_FLOOR * mean; published_curvatures[i] >= LOOP_FLOOR_GATE*mean && published_curvatures[i] < lo {
				published_curvatures[i] = lo
			}
		}
		// End BluePilot
	}

	// BluePilot: post-average boundary suppression. Flattening the INPUT
	// triples (above) is not enough: the 3-window average anchored on the jog
	// still mixes an unflattened neighbour, and because the jog's own arc
	// lengths are tiny (~13 m) that neighbour dominates the weighting. At the
	// northbound median crossover (36.1475565,-86.8162752) two flattened
	// samples plus one 0.018837 1/m neighbour published 0.013292 1/m — the
	// 12.27 m/s phantom that made up most of the map curve "targets" on the
	// route. Flatten the OUTPUT samples anchored on the jog nodes themselves.
	//
	// The jog is the pair of short boundary-adjacent segments b-1 -> b -> b+1
	// (that is exactly what boundarySegmentLength measured to classify it), so
	// the anchors to suppress are nodes b-1, b and b+1, i.e. output indices
	// b-3, b-2 and b-1. The output anchored at b+2 is left alone: that is
	// where the real curve the crossover sits on becomes visible (the 31st Ave
	// N crest curve peaks ~14 m past the crossover node).
	//
	// Applied last, after the max above, so nothing can resurrect it. Written
	// as a cap rather than an assignment so suppression can only ever raise a
	// published target, never lower one.
	for _, b := range crossover_nodes {
		for i := b - 3; i <= b-1; i++ {
			if i >= 0 && i < len(published_curvatures) && published_curvatures[i] > FLATTENED_CURVATURE {
				published_curvatures[i] = FLATTENED_CURVATURE
			}
		}
	}
	// End BluePilot

	curvature_outputs := make([]Curvature, len(published_curvatures))
	for i, curvature := range published_curvatures {
		curvature_outputs[i].Curvature = curvature
		curvature_outputs[i].Latitude = x_points[i+2]
		curvature_outputs[i].Longitude = y_points[i+2]
		// BluePilot: carries the per-way lateral-budget context to
		// GetTargetVelocities; never serialized
		curvature_outputs[i].IsRamp = point_is_ramp[i+2]
		// End BluePilot
	}
	return curvature_outputs, nil
}

type Velocity struct {
	Latitude  float64 `json:"latitude"`
	Longitude float64 `json:"longitude"`
	Velocity  float64 `json:"velocity"`
}

func GetTargetVelocities(curvatures []Curvature) []Velocity {
	velocities := make([]Velocity, len(curvatures))
	for i, curv := range curvatures {
		if curv.Curvature == 0 {
			continue
		}
		// BluePilot: the budget depends on the speed the curve is taken at,
		// so v = sqrt(a(v) * R) is implicit. a(v) is mild and decreasing, so
		// fixed-point iteration from the anchor-budget seed settles in a few
		// rounds (each step moves less than the one before; four rounds land
		// within ~0.1 m/s everywhere in the table's range).
		radius := 1.0 / curv.Curvature
		var v float64
		if curv.IsRamp {
			// latBudget's ramp branch is a min of two constants and never reads
			// v, so the fixed point is reached in one step and iterations 2-4
			// would recompute identical bits from identical inputs.
			v = math.Sqrt(latBudget(0, true) * radius)
		} else {
			v = math.Sqrt(TARGET_LAT_ACCEL * radius)
			for iter := 0; iter < 4; iter++ {
				v = math.Sqrt(latBudget(v, false) * radius)
			}
		}
		velocities[i].Velocity = v
		// End BluePilot
		velocities[i].Latitude = curv.Latitude
		velocities[i].Longitude = curv.Longitude
	}
	return velocities
}

func GetAverageCurvatures(curvatures []float64, arc_lengths []float64) ([]float64, error) {
	if len(curvatures) < 3 {
		return []float64{}, errors.New("not enough curvatures to average")
	}

	average_curvatures := make([]float64, len(curvatures)-2)

	for i := 0; i < len(curvatures)-2; i++ {
		a := curvatures[i]
		b := curvatures[i+1]
		c := curvatures[i+2]
		al := arc_lengths[i]
		bl := arc_lengths[i+1]
		cl := arc_lengths[i+2]

		if al+bl+cl == 0 {
			average_curvatures[i] = 0
			continue
		}

		average_curvatures[i] = (a*al + b*bl + c*cl) / (al + bl + cl)
	}

	return average_curvatures, nil
}

func GetCurvatures(x_points []float64, y_points []float64) ([]float64, []float64, error) {
	if len(x_points) < 3 {
		return []float64{}, []float64{}, errors.New("not enough points to calculate curvatures")
	}
	curvatures := make([]float64, len(x_points)-2)
	arc_lengths := make([]float64, len(x_points)-2)

	for i := 0; i < len(x_points)-2; i++ {
		curvature, arc_length, _ := GetCurvature(x_points[i], y_points[i], x_points[i+1], y_points[i+1], x_points[i+2], y_points[i+2])

		curvatures[i] = curvature

		arc_lengths[i] = arc_length
	}
	return curvatures, arc_lengths, nil
}

func GetCurvature(x_a float64, y_a float64, x_b float64, y_b float64, x_c float64, y_c float64) (float64, float64, float64) {
	length_a := DistanceToPoint(x_a*TO_RADIANS, y_a*TO_RADIANS, x_b*TO_RADIANS, y_b*TO_RADIANS)
	length_b := DistanceToPoint(x_a*TO_RADIANS, y_a*TO_RADIANS, x_c*TO_RADIANS, y_c*TO_RADIANS)
	length_c := DistanceToPoint(x_b*TO_RADIANS, y_b*TO_RADIANS, x_c*TO_RADIANS, y_c*TO_RADIANS)

	sp := (length_a + length_b + length_c) / 2

	area := math.Sqrt(sp * (sp - length_a) * (sp - length_b) * (sp - length_c))

	if length_a*length_b*length_c == 0 {
		return 0, 0, 0
	}

	curvature := (4 * area) / (length_a * length_b * length_c)

	radius := 1.0 / curvature

	angle := math.Acos((math.Pow(radius, 2)*2 - math.Pow(length_b, 2)) / (2 * math.Pow(radius, 2)))
	arc_length := radius * angle
	return curvature, arc_length, angle
}
