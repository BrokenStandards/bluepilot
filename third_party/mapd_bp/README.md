# mapd_bp — BluePilot fork of pfeiferj/openpilot-mapd

Fork base: **pfeiferj openpilot-mapd v1.12.0**. Fork version: **`v1.12.0-bp4`**
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

Some tests are empirical checks against a real generated tile and are skipped
unless `MAPD_BP_REAL_TILE` points at one:

```sh
MAPD_BP_REAL_TILE=/path/to/36.000000_-87.000000_36.250000_-86.750000 go test ./...
```

The endpoint-index cross-check sweeps every way in that tile against the
retained linear scan (~75 s); `-short` samples every 11th way instead (~7 s).

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

**bp4 refinement — peak-preserving curvature (`GetStateCurvatures`):** the
3-sample arc-length-weighted average in `GetAverageCurvatures` SMEARS
curvature peaks. At the 31st Ave N pre-light bend (36.1440162,-86.8165422) the
raw circumcircle triple centred on the bend gives R = 66.2 m, but the two
neighbouring triples (R = 318.1 m and R = 2543.8 m) carry comparable arc
weights (76.7 / 69.3 / 68.6 m) and dilute the published value to R = 166.0 m —
an 18.22 m/s (40.8 mph) target for a bend the driver takes at 54-67 m radius
and 2.3 m/s^2 lateral, braking manually in BOTH directions. The published
value is now the elementwise maximum of the averaged curvature and the RAW
curvature of the triple centred on the SAME node (`out[i] =
max(average_curvatures[i], curvatures[i+1])`, i.e. `R_out = min(R_avg,
R_raw_center)`), so the average can only soften the approach, never erase the
peak. It runs AFTER the merge/split and crossover writes into `curvatures[]`,
so deliberately flattened samples can never be resurrected.

The raw term is deliberately narrow, because a polyline is a coarse sampling
of a smooth road and an ungated max turns every sampling artifact into a
slowdown. Measured over the whole tile, an ungated max lowered 46.8% of all
anchors and cost a median 12.5 mph on ordinary curves and 17.5 mph on
interstates. Three gates confine it to the case it exists for
(`MIN_PEAK_ARC`, `MIN_PEAK_SAGITTA`, `MIN_PEAK_CURVATURE`):

- **arc >= 25 m** and **sagitta = arc²/8R >= 1.5 m.** A circumcircle only
  resolves a radius if the deviation it implies clears OSM's digitisation
  error. A straight stretch of Hillsboro Pike carries a 3.4 m node pair
  implying R = 39.7 m (0.15 m of sagitta) and turned a 45 mph road into an
  18 mph target; interstate corridors digitised at ~12 m spacing imply 0.24 m
  of sagitta at R = 300 m, i.e. pure noise.
- **R <= 120 m.** On gentle geometry the road's total turn lands unevenly on
  the nodes, and whichever node caught the largest share reads as a corner: on
  I 40 a corridor that runs at 569-851 m over a proper baseline turns 11.2° at
  one node, and the triple there honestly measures R = 227 m. Averaging is the
  right answer there. Below 120 m the sampling argument no longer holds — a
  bend that tight is a real feature, and it is exactly what the average smears
  into its straighter neighbours.

With the gates the same tile-wide sweep leaves ordinary curves and interstates
statistically indistinguishable from running the peak term off entirely
(median give-back 3.6 / 1.8 mph at Normal, matching the controller-only
reference), while the pre-light bend keeps its R = 66.2 m and the S-curve
bends keep theirs.

**bp4 refinement — post-average boundary suppression:** flattening the INPUT
triples at a crossover is not enough. The 3-window average anchored on the jog
still mixes an unflattened neighbour, and because the jog's own arc lengths
are tiny (~13 m) that neighbour dominates the weighting: northbound at the
south merge node (36.1475565,-86.8162752) two flattened samples plus one
0.018837 1/m neighbour published 0.013292 1/m — the 12.267 m/s phantom that
made up 79 of the 118 usable map targets on the whole route. The OUTPUT
samples anchored on the jog nodes themselves (`b-1`, `b`, `b+1`, i.e. output
indices `b-3 .. b-1`, since output `i` is anchored at `x_points[i+2]`) are now
capped at `FLATTENED_CURVATURE` after the max. The output anchored at `b+2` is
untouched: that is where the real curve the crossover sits on becomes visible.
Northbound merge-crossover target 12.27 -> 36.51 m/s; southbound crest curve
unchanged at 14.18 m/s; the well-modelled NB bridge inner curve unchanged at
13.13 m/s (R = 86.1 m).

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

### 5. Cap'n Proto read-traversal budget lifted for the tile (`src/tile_cache.go`)

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

**Fix:** `readOffline` (moved to `src/tile_cache.go` in bp3) calls
`msg.ResetReadLimit(math.MaxUint64)`
(go-capnp v3 alpha-29 API) right after a successful `UnmarshalPacked`,
making the budget effectively unlimited for that Message. This is safe
because the tile is generated locally by this same binary
(`--generate`) from Geofabrik data and stored on-device — it is trusted,
bounded, non-recursive data; the traversal budget only defends against
hostile REMOTE capnp payloads, which mapd never reads. Regression-tested by
running >20 full-tile `MatchingWays` scans followed by node reads on the
real tile.

### 6. Version constant

`VERSION = "v1.12.0-bp4"` in `src/mapd.go`, logged at startup.

## Performance (bp3)

bp3 contains **no behavioral change**. Every optimization below was accepted
only after the full recorded scenario corpus replayed **byte-identically**
against the pre-optimization binary (see *Equivalence proof*).

### What was optimized

**1. The unmarshalled tile is cached (`src/tile_cache.go`).**
`loop()` called `readOffline(state.Data)` every tick, re-running
`capnp.UnmarshalPacked` over the 2.97 MB packed tile — but `state.Data` is
only replaced when the car leaves the loaded tile's bounding box (31 times in
3030 recorded ticks, 1.0%). `readOffline` now keeps the `Offline` alongside
the `[]uint8` it came from and reuses it while that slice is the same slice.
The read-limit lift moved with it and is re-armed on every cache hit, so a
long-lived Message can never exhaust the traversal budget.

**2. `MatchingWays` uses an endpoint index (`src/tile_cache.go`, `src/way.go`).**
It was a linear scan over all 11037 ways, resolving every way's node list to
read two endpoints, and it is called once per graph hop — from `NextWay`
(66% of calls), the speed-limit-guess walk (22%) and `uTurnInAxisPoint`
(11%): p95 22 and up to 105 scans in a single 1 Hz tick, 70.66% of mapd's
total CPU. A map from each way's first/last node `(lat, lon)` to way indices
is now built once per tile load and looked up instead. It is keyed on exactly
the `float64` pair the scan compared with `==`, admits ways under the same
`HasNodes()`/`Len() >= 2` rules, and is built in tile order, so the candidate
list — including its order, which several `NextWay` tiers depend on — is
identical.

**3. Both caches invalidate by identity, never by content.** The tile cache
compares the backing array of the `[]uint8`; the index compares the
`*capnp.Message`. Each cache holds a reference to the object it is keyed on,
so that object cannot be collected and its address cannot be recycled — which
makes pointer equality a sound identity test. `InvalidateTileCaches()` is also
called from `loop()`'s panic-recovery path, where `state.Data` is discarded.

**4. Two file-descriptor leaks fixed (`src/params.go`).** `PutParam` never
closed the temp file it created nor the directory it opened to fsync, and
`RemoveParam` never closed its directory — ~10 leaked fds per 1 Hz tick,
reclaimed only by `os.File`'s finalizer, i.e. only by GC pressure. mapd
happened to generate that pressure by re-unmarshalling the tile every tick;
removing the garbage (item 1) turns the leak into a hard `EMFILE` crash. These
are pre-existing upstream defects, unrelated to any BluePilot feature.

**5. Redundant `MapAdvisoryLimit` write removed (`src/mapd.go`).** A bare
`float64` was written and then immediately overwritten by the `AdvisoryLimit`
object, so every tick had a window in which the param held a number instead of
the object its readers parse. No reader of the scalar form exists in the tree.

**Items 1 and 4 must ship together.** The fd leak was survivable only because
re-unmarshalling the tile forced roughly 1.6 GCs per tick, and it is the GC
that runs `os.File`'s finalizers. Cherry-picking the tile cache without the
`params.go` closes takes fds from a steady ~30 to 2622 over 120 ticks, i.e. an
`EMFILE` crash against a 1024 limit. Treat `tile_cache.go` and `params.go`
as one change.

**Tile bytes must be replaced, never rewritten in place.** Both caches key on
identity, so bytes refilled into the existing backing array would be served
stale. `FindWaysAroundLocation` returns a fresh `os.ReadFile` allocation, and
`LoadedTileBytes()` is the one sanctioned way to install them — it drops the
caches if it is ever handed the same array back, so a future buffer-reusing
producer degrades to a re-parse instead of returning the wrong tile.

### Measured, per stage

Full replay corpus: 34 recorded scenarios (Regions route SB/NB, 8 surface
forks, 16 interstate fork/ramp/gore runs, 5 speed-limit-guess corridors, 3
divergence-rematch variants), 3584 ticks against the real 11037-way Nashville
tile. Wall-clock figures are x86_64 (Xeon @ 2.10 GHz); **the comma 3X/4 ARM
cores are roughly 2-4x slower per core for this pointer-chasing work**, so
multiply accordingly. Prefer the ratios.

Steady-state ticks (n=3550, i.e. everything except the tile load), ms:

| stage | p50 before → after | p95 before → after | p99 before → after | max before → after |
| --- | --- | --- | --- | --- |
| **whole tick** | 19.39 → **0.061** | 85.74 → **0.262** | 128.85 → **0.916** | 360.99 → **1.26** |
| `readOffline` | 5.469 → **0.0001** | 10.79 → **0.0002** | 21.28 → **0.0003** | 49.83 → **0.020** |
| `NextWays` | 12.96 → **0.028** | 42.81 → **0.078** | 48.98 → **0.096** | 55.72 → **0.208** |
| `GetCurrentWay` | 0.0071 → **0.0030** | 0.030 → **0.018** | 0.863 → **0.825** | 1.373 → **1.196** |
| `ComputeSpeedLimitGuess`¹ | 65.00 → **0.083** | 146.56 → **0.230** | 287.90 → **0.386** | 315.52 → **0.447** |

¹ over the ticks that actually recompute the guess (way change or reload).

Allocation and GC over the same corpus:

| | before | after |
| --- | --- | --- |
| allocation per steady-state tick | 19.80 MB | **0.0058 MB** |
| total allocation over 3584 ticks | 70.98 GB | **0.74 GB** |
| GC cycles | 3881 | **104** |
| peak Go heap | 64.2 MB | **18.6 MB** |

Micro-benchmarks (warm process, real tile, `-benchtime 30x -count 3`):

| benchmark | before | after |
| --- | --- | --- |
| `readOffline`, tile unchanged | 9.34–10.19 ms, 19.80 MB, 55 allocs | **32.5–34.7 ns, 0 B, 0 allocs** |
| `MatchingWays`, one junction | 3.29–3.35 ms, 132 B | **0.59–0.61 µs, 128 B** (≈5500x) |
| endpoint index build | — | 5.35–5.46 ms, 1.30 MB, 16137 allocs, **once per tile load** |
| whole tile reload (unmarshal + index) | 9.34–10.19 ms **every tick** | 14.3–15.3 ms **on 1.0% of ticks** |

Live process (the shipped `mapd-x86_64` binary against the real tile with a
0.5 s GPS feed along the Regions route; 180 s, 175 ticks, identical behavior
in both — 61 way changes, 0 errors/warnings):

| | before | after |
| --- | --- | --- |
| CPU consumed | 10.76 s (6.15% of a core) | **0.29 s (0.17% of a core)** |
| open file descriptors | 27–75, sawtooth | **5, constant** |
| RSS | 149–168 MB | **85–94 MB** |

A 10-minute soak of the new binary (598 ticks) holds fds at 5 and RSS flat at
85.1–85.3 MB after the first GC, at 0.21% of a core — no growth from
retaining the unmarshalled tile.

On ARM this moves the worst recorded tick from roughly 0.7–1.4 s (i.e. past
the 1 s loop period, and a CPU-share neighbor of the real-time processes) to
roughly 2.5–5 ms, and steady-state mapd CPU from ~12–25% of a core to well
under 1%.

### Equivalence proof

`readOffline`-through-`GetTargetVelocities` was replayed tick by tick through
a harness that mirrors `loop()`'s call order and state exactly, compiled once
against the pristine bp2 tree and once against bp3, dumping the whole
published surface per tick: matched way (bbox, name, ref, lanes, oneway, node
count, direction, start/end), the full next-way chain, curvatures, target
velocities, the speed-limit guess, road name, `MapSpeedLimit`,
`NextMapSpeedLimit`, `MapAdvisoryLimit`, `NextMapAdvisoryLimit` and both
hazards.

- **38 output files, 4280 ticks, 35.4 MB of JSON — byte-identical.**
- Four of those runs replace `state.Data` with a freshly allocated copy of the
  tile every 7 ticks, exercising cache and index invalidation on 100 reloads;
  also byte-identical.
- `MatchingWays` was additionally cross-checked against the retained linear
  scan on **22074 endpoint probes** covering every way in the real tile
  (`TestMatchingWaysIndexMatchesLinearScanOnRealTile`).

### Deliberately NOT optimized

- **The guess walk's hop caps** (`GUESS_MAX_WAY_HOPS = 40`, both directions).
  Lowering to 16 would halve the worst-case scan count, but it is the only
  candidate that changes OUTPUT: 5.3% of currently-successful guesses (9 of
  169 measured walks, at 25–33 hops) would be lost, and skipping the forward
  walk would flip `source` from `both` to `backward` on ticks the UI's
  guessed-limit outline keys on. With the endpoint index the worst measured
  `ComputeSpeedLimitGuess` is 0.45 ms, so there is nothing left to buy.
- **Hoisting way endpoints into the tile schema** (`offline.capnp` +
  `generate_offline.go`). Same win as the index, but it is a tile FORMAT
  change requiring every tile to be regenerated and redistributed plus a
  compatibility path for tiles already on devices. Strictly dominated.
- **A per-tick `Way.Nodes()` memo.** Was 27.5% of CPU before the index; the
  index removes the repeated scans that caused it, so the memo would duplicate
  state capnp already owns for no remaining benefit.
- **`getPossibleWays`** is still a full-tile `OnWay` scan. It runs at most
  once per tick (only when the sticky match fails) and its bbox pre-filter
  keeps it at p99 0.83 ms / max 1.20 ms — it is now the largest single item
  left, and still two orders of magnitude inside the 1 s budget. A spatial
  index here would change which candidates are considered near tile edges;
  not worth the risk for ~1 ms.
- **The divergence rematch** (2 fires in 3030 real ticks, structurally capped
  at 1-per-3-ticks, 0.8 ms each), **`isUTurn`'s geometry test** (1.14 µs per
  candidate, 0.07% of CPU) and **the read-limit lift** (zero direct cost, and
  it is what makes reads after the 18th scan correct). Measured and cleared —
  recorded here so they are not re-litigated.

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
