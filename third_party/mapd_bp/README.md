# mapd_bp — BluePilot fork of pfeiferj/openpilot-mapd

Fork base: **pfeiferj openpilot-mapd v1.12.0**. Fork version: **`v1.12.0-bp1`**
(the `VERSION` file next to the binaries is the authoritative copy read by the
Python installer; the same string is compiled into the Go binary as the
`VERSION` constant in `src/mapd.go` and logged at startup).

Layout:

```
third_party/mapd_bp/
├── src/          Go source (package main, vendored + forked)
├── mapd          linux/arm64 static binary (comma device)
├── mapd-x86_64   linux/amd64 static binary (PC / CI)
├── VERSION       plain-text fork version
└── README.md     this file
```

## Rebuilding

```sh
scripts/build_mapd.sh
```

The script runs `go test ./...` in `src/`, then builds both binaries with
`CGO_ENABLED=0 GOOS=linux -ldflags "-s -w"` (GOARCH `arm64` → `mapd`,
`amd64` → `mapd-x86_64`). Requires Go >= 1.20 and module-proxy access.

## Behavioral changes vs v1.12.0

### 1. U-turn candidate rejection in `NextWay` (`src/way.go`)

**Defect fixed:** the same-name tier of `NextWay` happily chained onto the
opposite carriageway of divided roads (e.g. the 31st Ave N S-curve in
Nashville): the opposite carriageway ties the name comparison, the oneway
filter is topological only, and the junction-curvature check
(`isValidConnection`) cannot catch a hairpin whose three junction nodes lie on
a nearly flat circumcircle (~100 m legs, a few meters of lateral offset). The
resulting looped chains emitted phantom 7 m/s curve targets and continuous
phantom braking.

**Fix:** `isUTurn()` compares the bearing of the current way's final traversed
segment with the bearing of the candidate's first traversed segment (respecting
`NextIsForward`). Candidates whose bearing reverses by more than 150°
(`U_TURN_BEARING_DELTA_COS = -0.866`, i.e. `cos(delta) < cos(150°)`) are
rejected in ALL selection tiers (same-name, same-ref, split-ref, any-valid)
and in the last-resort `matchingWays` fallback. When the only same-name
continuation is a direction reversal, the name tier now falls through empty so
lower tiers can pick a renamed-but-straight continuation; if nothing but a
U-turn connects at all, `NextWay` returns no next way — a missing chain is
safer than a phantom one. Ordinary intersections are unaffected: chains keep
following the road/name exactly as before, minus direction reversals (no turn
prediction is attempted; the known wrong-branch-at-crossroads behavior is
accepted and out of scope).

### 2. Merge/split curvature flattening gated on known lane counts (`src/math.go`)

**Defect fixed:** `GetStateCurvatures` flattens curvature to 0.0015 1/m around
way boundaries it classifies as lane merges/splits, but classified `Lanes() ==
0` (untagged) as a real lane count. Every boundary between an untagged and a
tagged way "looked like" a lane change and had its REAL curve targets erased
(crest curve southbound / bottom bend northbound at the 31st Ave N carriageway
split).

**Fix:** the merge/split classification only applies when BOTH ways have known
lane counts (`Lanes() > 0` on both). Genuine tagged lane-count changes are
flattened exactly as before.

### 3. Speed-limit gap guess (`src/speed_limit_guess.go`)

**Defect fixed:** untagged maxspeed stretches (Charlotte Ave east of 22nd Ave
N, all of 31st Ave N) yielded speed limit 0 = "no limit" with no fallback.

**Fix:** when the current way's effective directional maxspeed is 0 and the
way has a name or ref, the road graph is walked in both directions along
connections whose name OR ref matches the current way, looking for the nearest
way with a tagged effective maxspeed (> 0; the directional tag matching the
travel direction on that way is preferred, else the generic tag):

- forward walk: traversal direction, normal oneway drivability rejection;
- backward walk: reverse traversal, oneway rejection skipped — we are sampling
  the road's signage zone, not routing — but name/ref continuity and U-turn
  rejection still apply.

Constants: `GUESS_MAX_SEARCH_DISTANCE = 3000` m and `GUESS_MAX_WAY_HOPS = 40`
per direction (bounds both road distance and CPU work on node-split roads).
Combination rules: backward-only → backward value/`"backward"`; forward-only →
forward value/`"forward"`; both equal → that value/`"both"`; both differing →
backward value/`"backward"` (the car most recently passed that signage zone;
the forward change still surfaces through the existing `NextMapSpeedLimit`
mechanism because `MapSpeedLimit` itself stays 0 — this fork never alters
`MapSpeedLimit`). The guess is cached in `State` and recomputed only when the
current way changes (`hasRoadInfoChanged`) or the offline tile data reloads;
it is published every loop on the `MapSpeedLimitGuess` mem param (reset to
`{}` in `ResetParams`).

### 4. Divergence-based rematch (`src/car_context.go`, hooks in `src/way.go` / `src/mapd.go`)

**Defect mitigated:** the sticky current-way match can lock onto the wrong way
(opposite carriageway / parallel road) and stay there while the car's actual
motion contradicts the map geometry.

**Mechanism:** the Python side (`osm_map_data`) publishes `MapdCarContext` at
1 Hz. When `enabled`, each loop compares |car curvature| with the matched
way's local curvature (`WayLocalCurvature`: node triple nearest the car,
computed with `GetCurvature`). A tick counts as diverged when the two disagree
by more than `DIVERGENCE_CURVATURE_DELTA = 0.004` 1/m AND at least one of them
exceeds `DIVERGENCE_MIN_CURVATURE = 0.005` 1/m (keeps straight-road noise from
accumulating). After `DIVERGENCE_TRIGGER_TICKS = 3` consecutive diverged
ticks, `GetCurrentWay` bypasses the sticky-match branch and runs full
candidate selection with an extra score term (`curvatureMatchScore`): linear
from `+CURVATURE_MATCH_MAX_SCORE = 30` at zero curvature difference down to
−30 at `CURVATURE_MATCH_SATURATION_DELTA = 0.01` 1/m, clamped to ±30 so it can
override the same-name stickiness bonus but not the road-hierarchy spread.
The rematch NEVER forces a switch: only a way that STRICTLY outscores the
current way (scored identically, including the curvature term) replaces it —
otherwise the normal sticky flow continues unchanged (fail safe). The tick
counter resets after each trigger so a failed rematch cannot re-run the
full-map scan every second.

### 5. Version constant

`VERSION = "v1.12.0-bp1"` in `src/mapd.go`, logged at startup.

## Mem-param interfaces

`MapdCarContext` — written by Python (`osm_map_data`) at 1 Hz, read by Go:

```json
{
  "enabled": true,            // mirror of the VisualRoutingAssist toggle
  "v_ego": 12.3,              // m/s
  "yaw_rate": 0.01,           // rad/s (liveLocationKalman angularVelocityCalibrated z)
  "curvature": 0.0008,        // 1/m, |yaw_rate|/max(v_ego,1) signed by yaw_rate
  "desired_curvature": 0.0    // 1/m, 0 if unavailable
}
```

`MapSpeedLimitGuess` — written by Go every loop, read by Python:

```json
{
  "speedlimit": 15.6464,      // m/s, 0 = no guess
  "source": "backward",       // "backward" | "forward" | "both" | ""
  "backward_value": 15.6464,  // m/s, 0 = none found
  "backward_distance": 0.0,   // m along the road to the tagged way behind
  "forward_value": 17.8816,   // m/s, 0 = none found
  "forward_distance": 240.0   // m along the road to the tagged way ahead
}
```

Distances are the sum of intermediate way lengths between the current way's
boundary node and the near end of the tagged way (0 when adjacent).

## Vendoring notes

- Vendored from the pfeiferj repo root: all `.go` files, `offline.capnp`,
  `go.mod`, `go.sum`, `LICENSE`, the nation/US-state bounding-box JSONs and the
  `.snapshots` test fixtures. Not vendored: `.git`, `cmd/dump`, docs,
  `Earthfile`, CI scripts.
- The `.snapshots` cupaloy fixtures for `TestVector`/`TestBearing` were
  regenerated: the current Go toolchain produces last-ulp (~1e-13 relative)
  floating-point differences vs the ones committed upstream.
- One incidental fix: `loop()` now re-reads the offline tile immediately after
  `FindWaysAroundLocation` reloads it, instead of finishing the tick on the
  stale tile; the reload-triggered speed-limit-guess recompute needs the fresh
  graph in the same tick.
