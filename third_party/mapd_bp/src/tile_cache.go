package main

// BluePilot: tile-level memoization for the 1 Hz mapd loop.
//
// Two caches live here. Neither changes what mapd computes — both replace
// repeated work with a lookup of exactly the same values.
//
//  1. The unmarshalled Offline for the current state.Data slice. loop() calls
//     readOffline(state.Data) every tick, but state.Data is only replaced when
//     the car leaves the loaded tile's bounding box (measured on the recorded
//     Nashville corpus: 31 replacements in 3030 ticks, 1.0%). Re-running
//     capnp.UnmarshalPacked on a 2.97 MB packed tile cost 5.15 ms and ~19.8 MB
//     of garbage per tick, which on its own forced a GC every ~3 ticks and
//     held a 163 MB peak RSS.
//
//  2. An endpoint index: first/last node coordinate -> way indices, so
//     MatchingWays looks its candidates up instead of scanning all 11037 ways.
//     That scan was 70.66% of mapd's CPU (3.24 ms per scan, p95 22 and up to
//     105 scans per tick through NextWay, uTurnInAxisPoint and the
//     speed-limit-guess walk).
//
// INVALIDATION is by identity, never by content:
//
//   - the Offline cache compares the backing array of the []uint8 it was
//     filled from, so a tile reload (FindWaysAroundLocation returns a freshly
//     allocated slice) always misses;
//   - the endpoint index compares the *capnp.Message it was built from, so it
//     can only ever be served to the exact tile it describes — including for
//     callers (tests, tools) that build an Offline without going through
//     readOffline.
//
// Pointer equality is a sound identity test here because each cache holds a
// reference to the object it is keyed on: that object cannot be collected, so
// its address cannot be recycled by a later allocation.

import (
	"math"

	"capnproto.org/go/capnp/v3"
	"github.com/pkg/errors"
)

// ---------------------------- offline cache ----------------------------

type offlineCacheEntry struct {
	data    []uint8
	offline Offline
	valid   bool
}

var offlineCache offlineCacheEntry

// sameTileBytes reports whether a and b are the same slice of the same
// underlying array. Zero-length slices never match: there is nothing to cache.
//
// This is an identity test, not a content test, so it holds only while tile
// bytes are replaced wholesale rather than rewritten in place. Every producer
// of tile bytes — FindWaysAroundLocation, via os.ReadFile — returns a freshly
// allocated slice, and LoadedTileBytes is the one sanctioned way to hand them
// over; a future producer that reuses a buffer must call InvalidateTileCaches
// instead of relying on the address changing.
func sameTileBytes(a, b []uint8) bool {
	return len(a) > 0 && len(a) == len(b) && &a[0] == &b[0]
}

// LoadedTileBytes records freshly loaded tile bytes as the current tile. It
// exists so the "replaced wholesale, never rewritten in place" invariant above
// has a single enforcement point: bytes that did not come from a new
// allocation drop the caches rather than being served stale.
func LoadedTileBytes(data []uint8) []uint8 {
	if offlineCache.valid && sameTileBytes(offlineCache.data, data) {
		InvalidateTileCaches()
	}
	return data
}

// readOffline returns the Offline root of the packed tile in data, reusing the
// previously unmarshalled message when data is the very same slice.
func readOffline(data []uint8) Offline {
	if offlineCache.valid && sameTileBytes(offlineCache.data, data) {
		// Re-arm the traversal budget so it can never be exhausted by a tile
		// that stays loaded for a long time (see the comment below on why the
		// budget is lifted at all). This is a single atomic store.
		if msg := capnp.Struct(offlineCache.offline).Message(); msg != nil {
			msg.ResetReadLimit(math.MaxUint64)
		}
		return offlineCache.offline
	}

	msg, err := capnp.UnmarshalPacked(data)
	logde(errors.Wrap(err, "could not unmarshal offline data"))
	if err != nil {
		return Offline{}
	}

	// Lift the per-Message capnp read-traversal budget (default 64 MiB per
	// Message). The budget exists to stop maliciously nested REMOTE payloads
	// from pinning the CPU, but this tile is locally generated trusted data
	// with bounded, non-recursive structure — the only thing the budget does
	// here is count every re-read, and a long-lived cached Message re-reads a
	// lot. MaxUint64 makes the budget effectively unlimited for its lifetime.
	msg.ResetReadLimit(math.MaxUint64)

	offline, err := ReadRootOffline(msg)
	logde(errors.Wrap(err, "could not read offline message"))
	if err != nil {
		return Offline{}
	}

	offlineCache = offlineCacheEntry{data: data, offline: offline, valid: true}
	return offline
}

// InvalidateTileCaches drops both caches and the memory they pin. Identity
// checking makes stale hits impossible on its own, so this is only needed to
// release a tile that is known to be dead (the panic-recovery path in loop()
// resets state.Data).
func InvalidateTileCaches() {
	offlineCache = offlineCacheEntry{}
	wayEndpoints = nil
}

// --------------------------- endpoint index ----------------------------

// endpointKey is the exact (lat, lon) float64 pair MatchingWays compares with
// ==, so an index keyed on it selects exactly the ways the linear scan would.
// NaN coordinates cannot exist in a generated tile; were one to appear it
// would be unreachable in the map for the same reason it never satisfies the
// scan's == comparison.
type endpointKey struct {
	lat, lon float64
}

type endpointIndex struct {
	msg   *capnp.Message
	byEnd map[endpointKey][]int32
	// first Nodes() read failure hit while indexing. The linear scan this
	// replaced surfaced such an error to its callers, which bail rather than
	// route on a truncated tile; the index reads the same nodes up front, so
	// it carries the error forward instead of silently skipping the way.
	buildErr error
}

var wayEndpoints *endpointIndex

// endpointIndexFor returns the endpoint index for offline, building it if the
// current one belongs to a different message. Build cost on the 11037-way
// Nashville tile: 6.9 ms and 1.8 MB, paid once per tile load.
func endpointIndexFor(offline Offline, ways capnp.StructList[Way]) (map[endpointKey][]int32, error) {
	msg := capnp.Struct(offline).Message()
	if msg == nil {
		return nil, nil
	}
	if wayEndpoints != nil && wayEndpoints.msg == msg {
		return wayEndpoints.byEnd, wayEndpoints.buildErr
	}

	var buildErr error
	byEnd := make(map[endpointKey][]int32, ways.Len())
	for i := 0; i < ways.Len(); i++ {
		w := ways.At(i)
		// Same admission rules as the linear scan in MatchingWays: a way with
		// no node list, or fewer than two nodes, can never match.
		if !w.HasNodes() {
			continue
		}
		nodes, err := w.Nodes()
		if err != nil {
			if buildErr == nil {
				buildErr = errors.Wrap(err, "could not read nodes from way")
			}
			continue
		}
		if nodes.Len() < 2 {
			continue
		}
		first := nodes.At(0)
		last := nodes.At(nodes.Len() - 1)
		firstKey := endpointKey{first.Latitude(), first.Longitude()}
		byEnd[firstKey] = append(byEnd[firstKey], int32(i))
		lastKey := endpointKey{last.Latitude(), last.Longitude()}
		if lastKey != firstKey {
			// A closed way whose ends coincide is listed once, matching the
			// scan's single append under its `first || last` condition.
			byEnd[lastKey] = append(byEnd[lastKey], int32(i))
		}
	}

	wayEndpoints = &endpointIndex{msg: msg, byEnd: byEnd, buildErr: buildErr}
	return byEnd, buildErr
}
