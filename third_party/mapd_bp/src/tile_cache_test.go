package main

// Tests for the tile-level caches in tile_cache.go. Both are pure performance
// caches, so the bar is: (1) a hit must return exactly what a cold call would,
// and (2) a hit must be impossible after the tile behind it changed. A stale
// endpoint index would silently chain the car onto ways from a tile it has
// already driven off, so the invalidation tests below are the important ones.

import (
	"os"
	"testing"

	capnp "capnproto.org/go/capnp/v3"
)

// matchingWaysLinearScan is the pre-index implementation of MatchingWays, kept
// here verbatim as the oracle the indexed version is checked against.
func matchingWaysLinearScan(currentWay Way, offline Offline, matchNode Coordinates) []Way {
	matchingWays := []Way{}
	ways, err := offline.Ways()
	if err != nil {
		return matchingWays
	}
	for i := 0; i < ways.Len(); i++ {
		w := ways.At(i)
		if !w.HasNodes() {
			continue
		}
		if w.MinLat() == currentWay.MinLat() && w.MaxLat() == currentWay.MaxLat() &&
			w.MinLon() == currentWay.MinLon() && w.MaxLon() == currentWay.MaxLon() {
			continue
		}
		wNodes, err := w.Nodes()
		if err != nil {
			return matchingWays
		}
		if wNodes.Len() < 2 {
			continue
		}
		fNode := wNodes.At(0)
		lNode := wNodes.At(wNodes.Len() - 1)
		if (fNode.Latitude() == matchNode.Latitude() && fNode.Longitude() == matchNode.Longitude()) ||
			(lNode.Latitude() == matchNode.Latitude() && lNode.Longitude() == matchNode.Longitude()) {
			matchingWays = append(matchingWays, w)
		}
	}
	return matchingWays
}

func sameWayList(a, b []Way) bool {
	if len(a) != len(b) {
		return false
	}
	for i := range a {
		if !isSameWay(a[i], b[i]) {
			return false
		}
	}
	return true
}

func wayNames(t *testing.T, ways []Way) []string {
	t.Helper()
	out := []string{}
	for _, w := range ways {
		n, _ := w.Name()
		out = append(out, n)
	}
	return out
}

// packOffline serializes a fixture tile the way generate_offline.go writes it,
// so readOffline can be exercised end to end.
func packOffline(t *testing.T, offline Offline) []uint8 {
	t.Helper()
	data, err := capnp.Struct(offline).Message().MarshalPacked()
	if err != nil {
		t.Fatalf("could not marshal fixture tile: %v", err)
	}
	return data
}

func freshCaches(t *testing.T) {
	t.Helper()
	InvalidateTileCaches()
	t.Cleanup(InvalidateTileCaches)
}

// Two tiles sharing a junction coordinate but continuing into differently
// named ways: reading one after the other must never mix them up.
func twoTilesSharingAJunction(t *testing.T) (Offline, Offline) {
	t.Helper()
	junctionLat, junctionLon := 36.10, -86.80
	tileA := buildOffline(t, []testWay{
		{name: "Shared Approach", lanes: 2, nodes: [][2]float64{{36.099, -86.802}, {junctionLat, junctionLon}}},
		{name: "Old Continuation", lanes: 2, nodes: [][2]float64{{junctionLat, junctionLon}, {36.101, -86.798}}},
	})
	tileB := buildOffline(t, []testWay{
		{name: "Shared Approach", lanes: 2, nodes: [][2]float64{{36.099, -86.802}, {junctionLat, junctionLon}}},
		{name: "New Continuation", lanes: 2, nodes: [][2]float64{{junctionLat, junctionLon}, {36.101, -86.798}}},
		{name: "New Spur", lanes: 2, nodes: [][2]float64{{junctionLat, junctionLon}, {36.100, -86.797}}},
	})
	return tileA, tileB
}

func junctionNode(t *testing.T, offline Offline, wayName string) Coordinates {
	t.Helper()
	nodes, err := wayByName(t, offline, wayName).Nodes()
	if err != nil {
		t.Fatalf("could not read nodes of %q: %v", wayName, err)
	}
	return nodes.At(nodes.Len() - 1)
}

func TestReadOfflineReusesUnmarshalledTile(t *testing.T) {
	freshCaches(t)
	tileA, _ := twoTilesSharingAJunction(t)
	data := packOffline(t, tileA)

	first := readOffline(data)
	second := readOffline(data)

	if capnp.Struct(first).Message() != capnp.Struct(second).Message() {
		t.Fatal("readOffline re-unmarshalled the same tile slice instead of reusing the cached message")
	}
	ways, err := second.Ways()
	if err != nil || ways.Len() != 2 {
		t.Fatalf("cached tile lost its ways: len=%d err=%v", ways.Len(), err)
	}
}

func TestReadOfflineInvalidatesWhenTileBytesReplaced(t *testing.T) {
	freshCaches(t)
	tileA, tileB := twoTilesSharingAJunction(t)
	dataA := packOffline(t, tileA)
	dataB := packOffline(t, tileB)

	readA := readOffline(dataA)
	waysA, _ := readA.Ways()
	if waysA.Len() != 2 {
		t.Fatalf("tile A: expected 2 ways, got %d", waysA.Len())
	}

	readB := readOffline(dataB)
	if capnp.Struct(readA).Message() == capnp.Struct(readB).Message() {
		t.Fatal("readOffline served the cached tile A message for tile B's bytes")
	}
	waysB, _ := readB.Ways()
	if waysB.Len() != 3 {
		t.Fatalf("tile B: expected 3 ways, got %d (stale tile A served)", waysB.Len())
	}
	if name, _ := waysB.At(1).Name(); name != "New Continuation" {
		t.Fatalf("tile B way 1 is %q, expected \"New Continuation\"", name)
	}

	// Back to A: identity is per-slice, so this must unmarshal A again and
	// still produce A's content.
	readAAgain := readOffline(dataA)
	waysA2, _ := readAAgain.Ways()
	if waysA2.Len() != 2 {
		t.Fatalf("tile A re-read: expected 2 ways, got %d", waysA2.Len())
	}
	if name, _ := waysA2.At(1).Name(); name != "Old Continuation" {
		t.Fatalf("tile A re-read way 1 is %q, expected \"Old Continuation\"", name)
	}
}

// A reload that happens to fetch byte-identical content still hands loop() a
// different slice; the cache must not treat the two as the same tile object.
func TestReadOfflineDoesNotMatchAnEqualButDistinctSlice(t *testing.T) {
	freshCaches(t)
	tileA, _ := twoTilesSharingAJunction(t)
	data := packOffline(t, tileA)
	copied := append([]uint8(nil), data...)

	first := readOffline(data)
	second := readOffline(copied)
	if capnp.Struct(first).Message() == capnp.Struct(second).Message() {
		t.Fatal("cache matched a distinct slice with equal content; identity check is not by backing array")
	}
	ways, err := second.Ways()
	if err != nil || ways.Len() != 2 {
		t.Fatalf("re-read of copied tile is broken: len=%d err=%v", ways.Len(), err)
	}
}

func TestInvalidateTileCachesForcesReload(t *testing.T) {
	freshCaches(t)
	tileA, _ := twoTilesSharingAJunction(t)
	data := packOffline(t, tileA)

	first := readOffline(data)
	InvalidateTileCaches()
	if offlineCache.valid || wayEndpoints != nil {
		t.Fatal("InvalidateTileCaches left cache state behind")
	}
	second := readOffline(data)
	if capnp.Struct(first).Message() == capnp.Struct(second).Message() {
		t.Fatal("readOffline reused the message after the cache was invalidated")
	}
}

// The stale-index failure mode: after driving onto a new tile, a junction
// coordinate that exists in BOTH tiles must resolve to the new tile's ways.
// Serving tile A's "Old Continuation" here would chain the car onto a way that
// is not in the loaded map.
func TestEndpointIndexInvalidatedOnTileSwap(t *testing.T) {
	freshCaches(t)
	tileA, tileB := twoTilesSharingAJunction(t)

	approachA := wayByName(t, tileA, "Shared Approach")
	nodeA := junctionNode(t, tileA, "Shared Approach")
	gotA, err := MatchingWays(approachA, tileA, nodeA)
	if err != nil {
		t.Fatalf("MatchingWays on tile A: %v", err)
	}
	if names := wayNames(t, gotA); len(names) != 1 || names[0] != "Old Continuation" {
		t.Fatalf("tile A junction resolved to %v, expected [Old Continuation]", names)
	}

	approachB := wayByName(t, tileB, "Shared Approach")
	nodeB := junctionNode(t, tileB, "Shared Approach")
	gotB, err := MatchingWays(approachB, tileB, nodeB)
	if err != nil {
		t.Fatalf("MatchingWays on tile B: %v", err)
	}
	names := wayNames(t, gotB)
	if len(names) != 2 || names[0] != "New Continuation" || names[1] != "New Spur" {
		t.Fatalf("tile B junction resolved to %v, expected [New Continuation New Spur] "+
			"(stale endpoint index from tile A)", names)
	}

	// And back again, to prove the index is not merely built once per process.
	gotA2, err := MatchingWays(approachA, tileA, nodeA)
	if err != nil {
		t.Fatalf("MatchingWays back on tile A: %v", err)
	}
	if names := wayNames(t, gotA2); len(names) != 1 || names[0] != "Old Continuation" {
		t.Fatalf("tile A re-query resolved to %v, expected [Old Continuation]", names)
	}
}

// NextWay is the consumer that matters: prove the whole chain step follows the
// currently loaded tile after a swap, not just MatchingWays.
func TestNextWayFollowsTheCurrentTileAfterSwap(t *testing.T) {
	freshCaches(t)
	tileA, tileB := twoTilesSharingAJunction(t)

	nextA, err := NextWay(wayByName(t, tileA, "Shared Approach"), tileA, true)
	if err != nil {
		t.Fatalf("NextWay on tile A: %v", err)
	}
	if name, _ := nextA.Way.Name(); name != "Old Continuation" {
		t.Fatalf("tile A next way is %q, expected \"Old Continuation\"", name)
	}

	nextB, err := NextWay(wayByName(t, tileB, "Shared Approach"), tileB, true)
	if err != nil {
		t.Fatalf("NextWay on tile B: %v", err)
	}
	if name, _ := nextB.Way.Name(); name != "New Continuation" {
		t.Fatalf("tile B next way is %q, expected \"New Continuation\" (stale index)", name)
	}
}

// The index must reproduce the linear scan exactly, including candidate order
// (several NextWay tiers take the first match) and the current-way exclusion.
func TestMatchingWaysIndexMatchesLinearScanOnFixture(t *testing.T) {
	freshCaches(t)
	shared := [2]float64{36.10, -86.80}
	offline := buildOffline(t, []testWay{
		{name: "A", lanes: 2, nodes: [][2]float64{{36.099, -86.801}, shared}},
		{name: "B", lanes: 2, nodes: [][2]float64{shared, {36.101, -86.799}}},
		{name: "C", lanes: 2, nodes: [][2]float64{{36.101, -86.801}, shared}},
		// Degenerate: single node, never matchable.
		{name: "D", lanes: 2, nodes: [][2]float64{shared}},
		// Closed loop starting and ending at the junction: listed once.
		{name: "E", lanes: 2, nodes: [][2]float64{shared, {36.102, -86.802}, shared}},
		// Touches the junction only mid-way: not an endpoint, never matched.
		{name: "F", lanes: 2, nodes: [][2]float64{{36.098, -86.804}, shared, {36.103, -86.796}}},
	})

	ways, err := offline.Ways()
	if err != nil {
		t.Fatalf("could not read fixture ways: %v", err)
	}
	for i := 0; i < ways.Len(); i++ {
		cur := ways.At(i)
		nodes, err := cur.Nodes()
		if err != nil || nodes.Len() == 0 {
			continue
		}
		for _, probe := range []Coordinates{nodes.At(0), nodes.At(nodes.Len() - 1)} {
			want := matchingWaysLinearScan(cur, offline, probe)
			got, err := MatchingWays(cur, offline, probe)
			if err != nil {
				t.Fatalf("MatchingWays: %v", err)
			}
			if !sameWayList(want, got) {
				t.Fatalf("way %d probe (%v,%v): index returned %v, linear scan returned %v",
					i, probe.Latitude(), probe.Longitude(), wayNames(t, got), wayNames(t, want))
			}
		}
	}
	// A coordinate no way touches must return an empty, non-nil slice.
	got, err := MatchingWays(ways.At(0), offline, junctionNode(t, offline, "B"))
	if err != nil {
		t.Fatalf("MatchingWays: %v", err)
	}
	if got == nil {
		t.Fatal("MatchingWays returned a nil slice")
	}
}

// Exhaustive cross-check against the real 11037-way tile: every endpoint of
// every way, both implementations, identical lists.
func TestMatchingWaysIndexMatchesLinearScanOnRealTile(t *testing.T) {
	if os.Getenv("MAPD_BP_REAL_TILE") == "" {
		t.Skip("MAPD_BP_REAL_TILE not set; skipping real-tile index cross-check")
	}
	freshCaches(t)
	offline := loadRealTile(t)
	ways, err := offline.Ways()
	if err != nil {
		t.Fatalf("could not read tile ways: %v", err)
	}

	// The oracle is the 3.2 ms linear scan, so the full sweep costs ~75 s.
	// -short samples every 11th way instead (~7 s) and still covers every
	// shape in the tile.
	stride := 1
	minProbes := 20000
	if testing.Short() {
		stride, minProbes = 11, 1800
	}

	checked := 0
	for i := 0; i < ways.Len(); i += stride {
		cur := ways.At(i)
		nodes, err := cur.Nodes()
		if err != nil || nodes.Len() < 2 {
			continue
		}
		for _, probe := range []Coordinates{nodes.At(0), nodes.At(nodes.Len() - 1)} {
			want := matchingWaysLinearScan(cur, offline, probe)
			got, err := MatchingWays(cur, offline, probe)
			if err != nil {
				t.Fatalf("MatchingWays: %v", err)
			}
			if !sameWayList(want, got) {
				t.Fatalf("way %d probe (%.9f,%.9f): index returned %d ways, linear scan returned %d",
					i, probe.Latitude(), probe.Longitude(), len(got), len(want))
			}
			checked++
		}
	}
	if checked < minProbes {
		t.Fatalf("only cross-checked %d probes; expected the full tile", checked)
	}
	t.Logf("cross-checked %d endpoint probes against the linear scan", checked)
}
