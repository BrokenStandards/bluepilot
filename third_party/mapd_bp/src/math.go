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
		lastWay = nextWay.Way
	}

	x_points := make([]float64, num_points)
	y_points := make([]float64, num_points)

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
		velocities[i].Velocity = math.Pow(TARGET_LAT_ACCEL/curv.Curvature, 1.0/2)
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
