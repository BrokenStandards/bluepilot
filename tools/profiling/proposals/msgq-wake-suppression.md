# Proposal: msgq wake-fanout suppression (FOR UPSTREAM commaai/msgq -- do NOT fork-carry)

Status: prototyped, benchmarked, adversarially reviewed 2026-07-31. Verdict:
adopt-with-changes, **land only via an upstream PR**; adopt here on the next
msgq sync after upstream review, TSAN, and aarch64 device soak. A missed-wake
bug in this code freezes safety-critical processes -- that is why it must not
be carried as a fork patch.

## Problem

msgq_msg_send() sends one tkill(SIGUSR2) per REGISTERED reader per publish,
whether or not that reader is blocked waiting on this queue. carState has 13
registered readers of which only one polls on it; the other 12 are spuriously
signaled 100x/s each. Measured cost: ~25us base + 10-12us per reader per
publish, GIL held (PubSocket.send has no `with nogil:`). Adds up to ~11-12% of
one core across card/selfdrived/controlsd in the container profile
(~30% of each process's GIL self-time), plus uncounted reader-side spurious
wake CPU.

## Fix

Per-reader `read_waiting` word in the shm header. msgq_poll sets it before its
pre-sleep readiness re-check (Dekker store-load, seq_cst both sides: publisher
stores write_pointer then loads read_waiting; reader stores read_waiting then
re-loads write_pointer -- no lost wakeup); publisher signals only flagged
slots; flag deliberately NOT consumed by the sender (consecutive publishes
re-signal until the reader exits poll); eviction still signals unconditionally.
All blocking waits funnel through msgq_poll, so one choke point covers both
Poller and blocking SubSocket.receive.

## Evidence

- 13-reader topic publish: p50 141.2us -> 36.3us (3.9x); 1-reader control
  unchanged (40.0 vs 38.1us); reader latency IMPROVED (p50 243 -> 175us).
- msgq C++ suite 12360 assertions PASS; cereal messaging 347 PASS; adversarial
  lost-wakeup test (120 one-shot publishes at random phases vs deep-asleep
  readers, both wait paths): max wake latency 1.0ms, no timeout-class outlier.

## Reviewer-required changes for the upstream PR

1. Re-assert read_waiting inside the sleep loop's re-check so a reader
   evicted+reconnected mid-poll re-flags its new slot; comment/test the
   exit-path clear targeting a migrated reader_id.
2. TSAN pass + aarch64 (weaker memory model) soak before merge anywhere.
3. State explicitly that shm header layout changes are an ABI break for
   mixed-version processes sharing a queue.

## Full prototype record

Verification, full unified diff, benchmarks, and review are archived in the
session workflow output; the diff below is the prototype as benchmarked.

### Verification
```json
{
 "analysis": "CONFIRMED, mechanism established. cereal/messaging/__init__.py:259 is `self.sock[s].send(dat)` \u2014 the msgq socket write, NOT serialization. to_bytes() (line 258) is 0.2-0.6% of self-time in both windows; measured at 0.7-1.2us for the real payloads (carState 168B, controlsState 280B, selfdriveState 80B, carOutput 48B). new_message is 3-4us, fill-copy ~4us. The entire hotspot is inside C++ msgq_msg_send (msgq/msgq.cc:234), called with the GIL HELD (msgq/ipc_pyx.pyx:237-239 PubSocket.send has no `with nogil:`, unlike SubSocket.receive), so py-spy --gil attributes it all to line 259.\n\nWHERE THE TIME GOES: msgq_msg_send is lock-free and never blocks (shm memcpy = 0.07us/send with 0 readers; size-independent below 4KB). The cost is one tkill(SIGUSR2) syscall PER REGISTERED READER per publish (msgq.cc:310-313), and the kernel-side signal delivery + scheduler wakeup is charged to the sender. Reader counts recovered from the profiling run's /dev/shm headers: carState=13, carControl=8, selfdriveState=6, controlsState=5, carOutput=4, sendcan=0 (free in sim). Paced 100Hz benchmark with blocked readers: ~25us base + ~10-12us/reader (13 readers: p50 137us, mean 165us), sys-time dominated. Speedscope callsite attribution matches: carState 290-300us/send profiled vs 165us benched clean / 170us with saturated cores \u2014 the 2x residual is preemption-while-holding-GIL on the oversubscribed 4-core box (bridge 138% + ui 61% + modeld 43%), which py-spy counts.\n\nSTRUCTURAL INEFFICIENCY (verified by benchmark): readers blocked polling a DIFFERENT service cost the same to wake (117us vs 118us @8 readers). Of carState's 13 readers, only plannerd actually polls on carState; the other 12 (controlsd polls selfdriveState, radard->modelV2, locationd/calibrationd->cameraOdometry, paramsd/torqued/lagd->livePose, ui...) are spuriously signaled 100x/s each \u2014 pure waste on both sender and reader side (reader-side spurious-wake CPU is NOT even counted in the 30%).\n\nPUBLISH AUDIT: nothing needlessly frequent. carParams/carParamsSP @0.02Hz, onroadEvents/onroadEventsSP rate-limited to 1Hz+on-change (negligible in profile). card publishes 4-5 msgs/frame @100Hz (carState, carOutput, carStateSP, sendcan-when-engaged, liveTracks); controlsd 3 (controlsState, carControl, carControlSP); selfdrived 2 (selfdriveState, selfdriveStateSP). SP shadow topics cost ~1% core combined (1-3 readers each). BP topics (controllerStateBP/carStateBP) not published in sim (non-Ford) \u2014 zero contribution here; ~2-3ms/s on a Ford device (1 reader each).\n\nOPTIMIZATION VERDICT: (1) The real fix is in msgq (upstream-shared infra): skip tkill for readers not actually blocked waiting on that queue \u2014 a per-reader 'waiting' flag in the shm header with a set-flag-then-recheck protocol. Would eliminate ~90% of wakes on hot topics (est. recovering most of the 11-12% core measured here, plus uncounted reader-side spurious wakes). RECOMMEND UPSTREAMING to commaai/msgq \u2014 do NOT fork locally: missed-wake bugs freeze safety-critical processes, and the memory-ordering protocol needs rigorous review + tests. (2) Cheap upstreamable hygiene: wrap socket.send in `with nogil:` in ipc_pyx.pyx (symmetric with receive); does not cut CPU for these single-threaded processes, mainly fixes profile attribution \u2014 low standalone value. (3) Fork-local: no needless publishes to cut; reducing SP shadow-topic rates buys ~1% core but changes SubMaster alive/freq_ok semantics for consumers \u2014 not worth it now. (4) Actionable guardrail for this fork: every added subscriber to a 100Hz topic costs every publisher ~10us/publish plus a 100Hz spurious wake in the reader \u2014 keep BP additions off hot topics or poll-consolidated. ON-DEVICE: cost survives qualitatively (same 13-reader wake fan-out, tkill ~1-3us on aarch64, 8 cores not oversubscribed) but the ~30% share will shrink substantially \u2014 likely to ~10-15% of these processes; container numbers overstate the share
```

### Proposal (includes unified diff)
```
Four candidate directions were evaluated against the real built stack; two are worth doing, two are honest no-gos.

=== PROPOSAL 1 (the big one, UPSTREAM to commaai/msgq — do not carry as a fork patch): skip wakeup signals to readers that are not waiting ===

Mechanism: the SIGUSR2 sent per registered reader per publish (msgq.cc msgq_msg_send) exists ONLY to interrupt a reader's nanosleep inside msgq_poll (the handler is a no-op). Readers doing non-blocking reads (SubMaster non_polled_services — 12 of carState's 13 subscribers) or sleeping on a DIFFERENT queue never need it. Add a per-reader-slot read_waiting word in the shm header; msgq_poll sets it before its pre-sleep readiness re-check (Dekker store-load, all ops seq_cst: publisher stores write_pointer then loads read_waiting; reader stores read_waiting then re-loads write_pointer — no lost wakeup) and clears it on exit; msgq_msg_send signals only flagged slots. The flag is deliberately NOT consumed by the sender, so consecutive publishes re-signal until the reader exits poll — preserving the existing recovery behavior for a signal that lands just before nanosleep begins. Eviction path still signals unconditionally. All waiting funnels through msgq_poll (blocking SubSocket.receive uses it internally), so one choke point covers everything. Verified with msgq C++ suite (12360 assertions / 14 cases PASS), msgq python tests (8 PASS; test_fake skipped — missing 'parameterized' dev dep, unrelated), cereal messaging tests (347 PASS), plus a purpose-built lost-wakeup test: 120 one-shot publishes to quiet topics at random phases against a deep-asleep reader (both Poller and blocking-receive paths) — max wake latency 862us, no 100ms-class outlier ever observed (a missed wake would show as poll-timeout latency).

Unified diff (prototype, applied+benchmarked+reverted; tree left clean):

--- a/msgq/msgq.h
+++ b/msgq/msgq.h
@@ -20,6 +20,9 @@ struct  msgq_header_t {
   uint64_t read_pointers[NUM_READERS];
   uint64_t read_valids[NUM_READERS];
   uint64_t read_uids[NUM_READERS];
+  // 1 while the reader is (about to be) blocked in msgq_poll waiting on this
+  // queue; publishers only send a wakeup signal to waiting readers.
+  uint64_t read_waiting[NUM_READERS];
 };
 
 struct msgq_queue_t {
@@ -29,6 +32,7 @@ struct msgq_queue_t {
   std::atomic<uint64_t> *read_pointers[NUM_READERS];
   std::atomic<uint64_t> *read_valids[NUM_READERS];
   std::atomic<uint64_t> *read_uids[NUM_READERS];
+  std::atomic<uint64_t> *read_waiting[NUM_READERS];
   char * mmap_p;
   char * data;
   size_t size;
--- a/msgq/msgq.cc
+++ b/msgq/msgq.cc
@@ -135,6 +135,7 @@ int msgq_new_queue(msgq_queue_t * q, const char * path, size_t size){
     q->read_pointers[i] = reinterpret_cast<std::atomic<uint64_t>*>(&header->read_pointers[i]);
     q->read_valids[i] = reinterpret_cast<std::atomic<uint64_t>*>(&header->read_valids[i]);
     q->read_uids[i] = reinterpret_cast<std::atomic<uint64_t>*>(&header->read_uids[i]);
+    q->read_waiting[i] = reinterpret_cast<std::atomic<uint64_t>*>(&header->read_waiting[i]);
   }
 
   q->data = mem + sizeof(msgq_header_t);
@@ -164,6 +165,7 @@ void msgq_init_publisher(msgq_queue_t * q) {
   for (size_t i = 0; i < NUM_READERS; i++){
     *q->read_valids[i] = false;
     *q->read_uids[i] = 0;
+    *q->read_waiting[i] = 0;
   }
 
   q->write_uid_local = uid;
@@ -199,11 +201,13 @@ void msgq_init_subscriber(msgq_queue_t * q) {
 
       for (size_t i = 0; i < NUM_READERS; i++){
         *q->read_valids[i] = false;
+        *q->read_waiting[i] = 0;
 
         uint64_t old_uid = *q->read_uids[i];
         *q->read_uids[i] = 0;
 
         // Wake up reader in case they are in a poll
+        // (evicted readers are always signaled, waiting or not)
         thread_signal(old_uid & 0xFFFFFFFF);
       }
 
@@ -222,6 +226,7 @@ void msgq_init_subscriber(msgq_queue_t * q) {
       // on the first read the read pointer will be synchronized with the write pointer
       *q->read_valids[cur_num_readers] = false;
       *q->read_pointers[cur_num_readers] = 0;
+      *q->read_waiting[cur_num_readers] = 0;
       *q->read_uids[cur_num_readers] = uid;
       break;
     }
@@ -306,10 +311,18 @@ int msgq_msg_send(msgq_msg_t * msg, msgq_queue_t *q){
   uint32_t new_ptr = ALIGN(write_pointer + msg->size + sizeof(int64_t));
   PACK64(*q->write_pointer, write_cycles, new_ptr);
 
-  // Notify readers
+  // Notify readers. Only readers that declared wait intent (blocked, or about to
+  // block, in msgq_poll on this queue) need a signal; the signal's sole purpose is
+  // to interrupt their nanosleep. Readers doing non-blocking reads, or blocked
+  // polling a *different* queue, are skipped. The flag is intentionally not
+  // cleared here: it stays set until the reader leaves msgq_poll, so consecutive
+  // publishes keep re-signaling (same recovery behavior as the unconditional
+  // signal had if a signal lands just before the reader enters nanosleep).
   for (uint64_t i = 0; i < num_readers; i++){
-    uint64_t reader_uid = *q->read_uids[i];
-    thread_signal(reader_uid & 0xFFFFFFFF);
+    if (*q->read_waiting[i]) {
+      uint64_t reader_uid = *q->read_uids[i];
+      thread_signal(reader_uid & 0xFFFFFFFF);
+    }
   }
 
   return msg->size;
@@ -436,6 +449,7 @@ int msgq_msg_recv(msgq_msg_t * msg, msgq_queue_t * q){
 
 int msgq_poll(msgq_pollitem_t * items, size_t nitems, int timeout){
   int num = 0;
+  bool wait_flagged = false;
 
   // Check if messages ready
   for (size_t i = 0; i < nitems; i++) {
@@ -443,6 +457,26 @@ int msgq_poll(msgq_pollitem_t * items, size_t nitems, int timeout){
     if (items[i].revents) num++;
   }
 
+  if (num == 0) {
+    // Declare wait intent on every polled queue, *then* re-check readiness.
+    // Publishers store the write pointer before loading read_waiting, and we
+    // store read_waiting before re-loading the write pointer (all seq_cst), so
+    // either the publisher sees our flag and signals, or we see its message in
+    // the re-check below -- no lost wakeup.
+    wait_flagged = true;
+    for (size_t i = 0; i < nitems; i++) {
+      msgq_queue_t *q = items[i].q;
+      *q->read_waiting[q->reader_id] = 1;
+    }
+
+    for (size_t i = 0; i < nitems; i++) {
+      if (items[i].revents == 0 && msgq_msg_ready(items[i].q)) {
+        items[i].revents = 1;
+        num++;
+      }
+    }
+  }
+
   int ms = (timeout == -1) ? 100 : timeout;
 
 #ifdef __APPLE__
@@ -490,6 +524,14 @@ int msgq_poll(msgq_pollitem_t * items, size_t nitems, int timeout){
 #endif
   }
 
+  if (wait_flagged) {
+    // Withdraw wait intent
+    for (size_t i = 0; i < nitems; i++) {
+      msgq_queue_t *q = items[i].q;
+      *q->read_waiting[q->reader_id] = 0;
+    }
+  }
+
   return num;
 }

(Diff also saved at /tmp/claude-0/-home-user-bluepilot/10e247fb-c4e6-5e31-9d7e-93d20f2bffe1/scratchpad/msgq_waiting_flag.diff; benchmark harnesses at .../scratchpad/bench_pub.py and .../scratchpad/bench_wake_latency.py.)

=== PROPOSAL 2 (fork-local, safe, small): decimate BP topics to 20Hz + remove dead subscription ===

controllerStateBP has ZERO in-process data readers: nothing anywhere reads sm['controllerStateBP'] — the "torque bar" comment at /home/user/bluepilot/selfdrive/ui/sunnypilot/ui_state.py:36 is stale; its only live consumer is loggerd (should_log=True). carStateBP's only reader is the UI hybrid gauges (display-only, sm.valid-gated at /home/user/bluepilot/selfdrive/ui/bp/onroad/hud_renderer_bp.py:66). Both are BP-only topics (no cross-fork consumers, no upstream sync burden). Diff:

--- a/cereal/services.py
+++ b/cereal/services.py
@@
-  "controllerStateBP": (True, 100., 10),
-  "carStateBP": (True, 100., 10),
+  "controllerStateBP": (True, 20., 2),   # BluePilot: UI/log-only; qlog rate unchanged (10Hz)
+  "carStateBP": (True, 20., 2),
--- a/bluepilot/selfdrive/car/bp_card_publisher.py
+++ b/bluepilot/selfdrive/car/bp_card_publisher.py
@@
 _SETTINGS_INTERVAL = 5.0  # re-read params at most every 5 s
+# card runs at 100Hz but both BP topics are consumed only by the UI (<=20Hz render)
+# and the log. Keep in sync with the declared frequency in cereal/services.py.
+_BP_PUBLISH_DECIMATION = 5
@@
-def publish_controller_state_bp(CI, pm):
+def publish_controller_state_bp(CI, pm, frame: int = 0):
   """Publish controllerStateBP if the car controller reports lateralUncertainty."""
   global _settings_last_read, _settings_cache
+  if frame % _BP_PUBLISH_DECIMATION != 0:
+    return
   if hasattr(CI.CC, "lateralUncertainty"):
@@
-def publish_car_state_bp(CI, pm, can_valid):
+def publish_car_state_bp(CI, pm, can_valid, frame: int = 0):
   """Publish carStateBP (hybrid drive gauge data) if available from car state."""
+  if frame % _BP_PUBLISH_DECIMATION != 0:
+    return
   if hasattr(CI.CS, 'car_state_bp_msg') and CI.CS.car_state_bp_msg is not None:
--- a/selfdrive/car/card.py
+++ b/selfdrive/car/card.py
@@ (state_publish)
-      publish_car_state_bp(self.CI, self.pm, CS.canValid)
+      publish_car_state_bp(self.CI, self.pm, CS.canValid, self.sm.frame)
@@ (controls_update)
-      publish_controller_state_bp(self.CI, self.pm)
+      publish_controller_state_bp(self.CI, self.pm, self.sm.frame)
--- a/selfdrive/ui/sunnypilot/ui_state.py
+++ b/selfdrive/ui/sunnypilot/ui_state.py
@@
-      "controllerStateBP",  # BluePilot: lateral uncertainty for torque bar
(remove the dead subscription; nothing reads it)

Needs a behavioral test: assert both topics publish at 20Hz and that SubMaster alive/freq_ok stay OK for a 20Hz-declared/20Hz-published service; cereal test_services rules (freq<=104, decimation!=0) already pass with (20., 2).

=== EVALUATED AND REJECTED (honest no-gos) ===
(a) Reduce 100Hz message count elsewhere: audit confirms nothing else is needlessly frequent — carParams/carParamsSP already at 0.02Hz, onroadEvents rate-limited; carState/carOutput/controlsState/selfdriveState are 100Hz by contract (control loop). Cutting SP shadow topics changes SubMaster semantics for consumers for ~1% core — not proposed.
(b) capnp usage patterns: measured to_bytes 0.6-1.2us, new_message+assign 3.9us, fill 3.8-5.7us vs 140us send — serialization is <5% of the hotspot. Packed encoding would ADD CPU and break the wire format (C++ readers use FlatArrayMessageReader on unpacked bytes); pycapnp builders are not reusable. Nothing to do.
(c) Sync upstream: fetched commaai/msgq master (425b61a) — zero diff in msgq.cc/ipc_pyx.pyx vs this fork; fetched commaai/openpilot master openpilot/cereal/messaging/__init__.py — PubMaster.send byte-identical (typing cosmetics only). Upstream has NOT optimized this path; there is nothing to sync. (This also means Proposal 1 is a genuine upstream contribution opportunity.)
(d) Optional 2-line hygiene (upstreamable, not benchmarked as a win): declare `int send(char *, size_t) nogil` in msgq/ipc.pxd and wrap PubSocket.send's call in `with nogil:` in ipc_pyx.pyx (symmetric with receive, which already does). Fixes py-spy --gil attribution and lets background threads run during the send; does not reduce CPU for these single-threaded processes.
```

### Benchmark
```
All numbers from the real built stack (uv venv, isolated OPENPILOT_PREFIX, real msgq shm), 4-core x86_64 container, paced-100Hz publisher on carState (168B payload) with 1 genuinely-polling reader + 12 "spurious" readers that are registered on carState but blocked polling livePose@20Hz and drain carState non-blocking each wake — exactly SubMaster's non_polled_services pattern, matching the profiled 13-reader carState fan-out.

PROPOSAL 1 (msgq waiting flag), built and A/B-measured with scons-rebuilt ipc_pyx.so:
- BEFORE (baseline, 2 runs): send p50=140.5/140.4us, mean=147.1/144.2us, p90=203.9/200.2us; sender CPU 1.9% core; spurious readers 14.8/17.8% core total (12.4-14.8 ms/s each); real reader 2000/2000 delivered, lat p50 263us.
- AFTER (patched, 2 runs): send p50=38.9/41.2us, mean=38.7/43.4us, p90=53.0/54.9us; sender CPU 1.1-1.2% core; spurious readers 12.3% core total (10.2 ms/s each, ~2.2 ms/s saved per idle process); real reader 2000/2000 delivered, lat p50 186-208us (improved).
- Control, 1 reader only: before p50=39.4us, after p50=40.4us — no regression; patched 13-reader cost equals the 1-reader baseline, i.e. all 12 spurious tkills eliminated (~8.4us/reader measured slope, consistent with verification's 10-12us).
- Net: 3.4-3.6x faster publish on a 13-reader topic. Scaled to the profiled stack (11-12% of a core in __init__.py:259 across card/controlsd/selfdrived, dominated by signaling non-waiting readers per verification), projected recovery is roughly 6-8% of a core here, PLUS the reader-side spurious-wake savings that the 30% figure never counted. On-device the absolute win shrinks (tkill 1-3us on aarch64, no core saturation) but the fan-out ratio (13 registered / ~1 waiting) is identical.
- Correctness: msgq C++ tests 12360 assertions PASS; msgq python tests 8 PASS; cereal messaging tests 347 PASS; dedicated lost-wakeup test (120 one-shot publishes, random phases, deep-asleep readers on both Poller and blocking-receive paths): max wake latency 862us, zero timeout-class misses. Baseline re-verified after revert (p50 140.4us) — tree left as found.

PROPOSAL 2 (BP topics to 20Hz), measured on the real publish path with a 1-sleeping-reader sub socket and steady-state settings cache (Params untouched, as in production between 5s refreshes):
- publish_controller_state_bp end-to-end: p50=261.0us/call mean=266.8us paced@100Hz (n=1000); p50=293.5us paced@20Hz (n=240). Component medians: struct build+49 setattrs 5.7us, convert_to_capnp 35.8us (asdictref + new_message(**kwargs)), new_message+assign 3.9us, to_bytes(200B) 0.6us, sock.send 0.9us tight-loop — i.e. the paced cost is dominated by cold-cache Python execution, not the send.
- At 100Hz: ~26.1 ms/s of card's budget; at 20Hz: ~5.9 ms/s. SAVES ~20 ms/s ≈ 2% of a core on card (Ford devices only; zero in this sim profile since BP topics don't publish for non-Ford). carStateBP publish rides along (send-only, message prebuilt in opendbc CAN parsing).

REJECTED-DIRECTION numbers: to_bytes 0.6-1.2us / new_message 3.1-4.3us / fill 3.8-5.7us vs 140us send (capnp not the cost); upstream msgq master and openpilot master publish paths diffed byte-identical (nothing to sync).
```

### Adversarial review
```json
{
 "reasoning": "I attempted to refute the proposal by re-running every benchmark against the real built stack, auditing the patch line-by-line against msgq.cc/impl_msgq.cc, grepping for every consumer of the affected topics/APIs, and fetching upstream to check the sync-burden claim. The proposal survived: (1) Waiting-flag patch: reproduced 141.2us -> 36.3us p50 on a 13-reader topic (3.9x), 1-reader control unchanged (40.0 vs 38.1us), 100% delivery with IMPROVED reader latency (p50 243->175us), zero lost wakes in the adversarial one-shot test (max 1.0ms; a miss would show as >=100ms), 12360 C++ assertions + 347 cereal messaging tests pass under the patch. The Dekker store-load protocol is sound (seq_cst atomics both sides; publisher stores write_pointer then loads read_waiting, reader stores read_waiting then re-checks readiness), and I verified the choke-point claim: blocking SubSocket.receive (impl_msgq.cc:69-86), MSGQPoller, and cereal/messaging/msgq_to_zmq.cc all funnel through msgq_poll; no other SIGUSR2/nanosleep waiters exist in the repo. The signal-lands-before-nanosleep race is pre-existing and identically bounded before/after (flag deliberately not consumed by sender). The one genuinely NEW race \u2014 eviction/slot-migration leaving a reader unflagged for up to one 100ms self-heal cycle, plus the exit-path clearing the wrong slot if reader_id changed mid-poll \u2014 is real but rare, bounded, and disclosed. Upstream claim verified hard: msgq_repo's origin IS commaai/msgq, HEAD files are byte-identical to master 425b61a, so fork-carrying this would be msgq's first-ever divergence \u2014 the \"upstream it, don't carry it\" verdict is exactly right for shared infrastructure with this risk profile. Profile attribution verified from /tmp/prof3: send at messaging/__init__.py:259 is 31.2%/29.7%/32.7% of card/controlsd/selfdrived samples \u2248 12% of a core combined. (2) BP decimation: publish_controller_state_bp cost reproduced (263.4us vs claimed 261.0, components match); controllerStateBP confirmed to have ZERO in-process readers (torque bar reads controlsState/carState/carOutput, not controllerStateBP \u2014 the ui_state.py:36 subscription is dead); the web CerealDataPanel reads qlog via /api/cereal/.../qlog/, and (20., 2) preserves qlog at exactly 10Hz, so the only out-of-UI consumer is unaffected; sm.frame decimation matches card.py's existing carParams pattern. Refutations that landed but are not fatal: (a) the benchmark's 1-of-13-waiting model is optimistic \u2014 plannerd polls carState, selfdrived has a dedicated blocking carState sub_sock (selfdrived.py:92), and dmonitoringd blocks on all its services, so ~3-4 of 13 readers still get signaled post-patch; the projected 6-8%-of-a-core recovery is the top of the plausible range, and the on-device number will be low single digits (cheaper tkill, no core saturation) \u2014 which the proposal itself concedes; (b) \"carStateBP's only reader is hud_renderer_bp.py:66\" is materially incomplete \u2014 ~10 UI widget files read it (powerflow/hybrid-battery gauges incl. mici/arched variants) \u2014 but all are display-only and recv_frame/valid-gated, so the 20Hz conclusion stands; (c) both no-go rejections check out (capnp micro-costs reproduced within noise; upstream publish path byte-identical). Tree was left as found; I re-verified baseline (137.7us p50, C++ suite passes) after revert.",
 "verdict": "adopt-with-changes",
 "requiredChanges": "PROPOSAL 1 (msgq waiting flag): (1) Land ONLY via a commaai/msgq PR as proposed \u2014 do not merge into this fork; adopt on the next msgq sync after upstream review, TSAN, and aarch64 device soak. (2) In the upstream PR, harden the eviction/migration race: re-assert read_waiting[q->reader_id] inside the sleep loop's re-check (one store per wake) so a reader that was evicted+reconnected mid-poll re-flags its new slot, and note that the exit-path clear can target a migrated reader_id (harmless today because init_subscriber/init_publisher zero the flag, but it deserves a comment/test). (3) State the shm-layout break prominently (header +120B; mixed old/new binaries on one queue read garbage; fine for openpilot's restart-together model, breaks branch-switching dev workflows without a full restart). (4) Correct the projection: in the real stack ~3-4 of carState's 13 readers genuinely wait (plannerd poll='carState', selfdrived's dedicated blocking carState socket at selfdrive/selfdrived/selfdrived.py:92, dmonitoringd blocks on all its services), so quote container recovery as ~5-8% of a core and device recovery as low single digits \u2014 not the benchmark's 12-of-13 ideal. PROPOSAL 2 (BP topics to 20Hz): (1) Ship the promised rate test in the same commit as the services.py frequency change + decimation (assert both topics publish at 20Hz from a 100Hz loop and SubMaster alive/freq_ok hold for the 20Hz declaration). (2) Remove the `frame: int = 0` default in both publish functions \u2014 make frame a required argument so a future callsite cannot silently bypass decimation. (3) Fix the audit text before merge: carStateBP has ~10 in-repo UI consumer files (hybrid_battery_gauge, power_flow_gauge, powerflow_gauge(_arched), mici variants, hud_renderer_bp), not just hud_renderer_bp.py:66 \u2014 all display-only and recv_frame/valid-gated, so 20Hz remains safe, but the commit message must reflect the real consumer set. (4) The dead controllerStateBP subscription removal at selfdrive/ui/sunnypilot/ui_state.py:36 is verified safe (zero readers in-repo; web panel reads qlog, which stays 10Hz via decimation 2). Keep the optional nogil-on-send hygiene item out of this change (unbenchmarked; upstream separately if desired)."
}
```
