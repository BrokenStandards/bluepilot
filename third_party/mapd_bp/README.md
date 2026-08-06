# mapd_bp — BluePilot fork of pfeiferj/openpilot-mapd

Fork base: **pfeiferj openpilot-mapd v1.12.0**. Fork version: **`v1.12.0-bp2`**
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
`CGO_ENABLED=0 GOOS=linux -trimpath -buildvcs=false -ldflags "-s -w"`
(GOARCH `arm64` → `mapd`, `amd64` → `mapd-x86_64`). Requires Go >= 1.20 and
module-proxy access.

**Reproducibility:** `-trimpath` strips the local build directory from
embedded paths and `-buildvcs=false` drops VCS stamping, so rebuilding the
same source with the same Go toolchain produces byte-identical binaries
regardless of checkout location or git state (verified: two consecutive
`build_mapd.sh` runs produce identical sha256s for both binaries). To audit a
release, rebuild and compare `sha256sum mapd mapd-x86_64`.

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

**Fix:** `isUTurn()` rejects candidates whose continuation reverses the road
axis by more than 150° (`U_TURN_BEARING_DELTA_COS = -0.866`, i.e. `cos(delta)
< cos(150°)`), in ALL selection tiers (same-name, same-ref, split-ref,
any-valid) and in the last-resort `matchingWays` fallback. When the only
same-name continuation is a direction reversal, the name tier now falls
through empty so lower tiers can pick a renamed-but-straight continuation; if
nothing but a U-turn connects at all, `NextWay` returns no next way — a
missing chain is safer than a phantom one. Ordinary intersections are
unaffected: chains keep following the road/name exactly as before, minus
direction reversals (no turn prediction is attempted; the known
wrong-branch-at-crossroads behavior is accepted and out of scope).

**bp2 refinement — road-axis bearings:** bp1 measured the reversal from the
two single segments adjacent to the junction node, which under-measures it at
divided-carriageway *crossover* junctions: the ~9 m median-crossover jog
segments rotate ~30-45° toward each other, so the true 180° carriageway
reversal measured only ~117-127° (cos −0.449 at the 31st Ave N south merge
node for the SB→NB carriageway hop; cos −0.601 at the crest) and slipped past
the threshold. bp2 measures ROAD-AXIS chord bearings instead: each side of
the junction is walked at least `U_TURN_AXIS_DIST = 30` m of along-road
distance (clamped at the way end for the candidate side) and the bearing is
taken over the chord junction↔reached point, swallowing the jog. When the
from-way itself is shorter than 30 m (carriageways are chopped into tiny ways
right at crossovers, e.g. the 10.8 m crest stub), the approach-axis walk
continues into the unique connected same-name way behind it, up to
`U_TURN_AXIS_MAX_HOPS = 2` extra ways; any ambiguity clamps the walk
conservatively. Measured on the Nashville tile: south merge SB→NB now 169.8°
(cos −0.984, rejected) while SB→straight-stub is 5.8° (accepted); crest
NB→SB now 169.4° (cos −0.983, rejected) while NB→28th-Avenue is 5.1°
(accepted). 90° corners still measure ~90° and pass.

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

**bp2 refinement — median-crossover jog suppression:** removing the
unknown-lane flattening exposed a different artifact at divided-carriageway
crossovers: the short (< 15 m) node-split jog segments where a two-way way
hands over to a oneway carriageway fake a hard bend that no traffic drives.
Southbound over the 31st Ave N crest the approach minimum became 9.59 m/s at
the crossover node (36.14980,-86.81790) while the real crest curve is
14.18 m/s ~14 m away. bp2 detects the *crossover signature* — a way boundary
that switches between two-way and oneway AND whose boundary-adjacent traversal
segments are both shorter than `CROSSOVER_MAX_BOUNDARY_SEGMENT = 15` m — and
flattens ONLY the two curvature samples whose triples lie entirely inside the
jog (those centered on the boundary node and the node immediately before it),
NOT the 15 m sweep the merge/split path uses, which would re-mask the real
curve. Verified on the tile southbound chain (28th Avenue → SB carriageway):
crossover-node target 9.59 → 12.99 m/s, crest-curve target unchanged at
14.18 m/s.

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

**bp2 hardening of the walk:** (a) each directional walk keeps a visited set
keyed by the bbox-tuple way identity used for way equality elsewhere
(`isSameWay`), so loops and roundabouts of same-name ways can no longer cycle
the walk until the hop cap; (b) when multiple same-name/ref candidates connect
at a hop node (e.g. a same-named side spur), the walk continues onto the one
with the smallest continuation bearing change instead of first-in-tile-order;
(c) the backward walk runs before the forward walk — backward wins value
conflicts, so if any per-tick work budget runs out it is the forward walk that
starves.
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
full-map scan every second. Since bp2 the tick counter ALSO resets whenever
the matched way changes (same branch that resets `StableWayCounter`), so a
divergence streak never spans two different match hypotheses.

**Known limitation (measured, accepted):** on gently-diverging interstate
exits — measured ramp curvature 0.002-0.0045 1/m over the first ~200 m of the
ramp — the divergence rematch does not beat normal stickiness: the
car-vs-map curvature disagreement stays at or below
`DIVERGENCE_CURVATURE_DELTA` and/or under `DIVERGENCE_MIN_CURVATURE`, and
even when ticks accumulate the curvature score term is too small to strictly
outscore the mainline. This is deliberate — the thresholds are conservative
so the rematch can never be triggered by sensor noise or lane changes; the
feature is aimed at (and helps on) sharper divergences such as opposite
carriageways and forks with a real geometry difference.

**Cost profile:** every triggered rematch runs a full candidate selection
over the tile (`getPossibleWays` scans all ways; ~11k ways / ~3.6 MiB of
capnp traversal per scan on the Nashville tile). When the context signal
*persistently* disagrees with the map (e.g. long construction detour, map
error), the trigger re-fires at most once every `DIVERGENCE_TRIGGER_TICKS =
3` seconds, i.e. one extra full scan every 3 ticks on top of the normal
per-tick work, indefinitely, until the disagreement clears or a better way
wins. That steady-state cost is bounded and was measured as acceptable on
device, but is worth knowing about when profiling mapd CPU.

### 5. Cap'n Proto read-traversal budget lifted for the tile (`src/mapd.go`)

**Defect fixed (bp2):** go-capnp gives every `Message` a read-traversal
budget (default 64 MiB) as a defense against maliciously nested payloads:
every byte *traversed* counts against it, including re-reads. mapd re-reads
way node lists constantly — one `MatchingWays` graph hop re-reads every
way's nodes (~3.6 MiB per scan on the 11037-way Nashville tile) — so the
speed-limit-guess walk alone burned the budget after ~18 scans. Once
exhausted, every later read on the same `Message` silently returns empty
structs: `GetStateCurvatures`, `RoadName` and `NextMapSpeedLimit` blanked
for the rest of the tick (verification observed 14/109 ticks publishing
EMPTY `MapTargetVelocities`, causing brake pulsing), and the guess itself
silently zeroed.

**Fix:** `readOffline` calls `msg.ResetReadLimit(math.MaxUint64)`
(go-capnp v3 alpha-29 API) right after a successful `UnmarshalPacked`,
making the budget effectively unlimited for that Message. This is safe
because the tile is generated locally by this same binary
(`--generate`) from Geofabrik data and stored on-device — it is trusted,
bounded, non-recursive data; the traversal budget only defends against
hostile REMOTE capnp payloads, which mapd never reads. Regression-tested by
running >20 full-tile `MatchingWays` scans followed by node reads on the
real tile.

### 6. Version constant

`VERSION = "v1.12.0-bp2"` in `src/mapd.go`, logged at startup.

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
