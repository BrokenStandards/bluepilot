#!/usr/bin/env python3
import math
import numpy as np

import cereal.messaging as messaging
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, LongitudinalPlanSource
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.common.swaglog import cloudlog

from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP, classify_primary_limiter

A_CRUISE_MAX_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
ALLOW_THROTTLE_THRESHOLD = 0.4
MIN_ALLOW_THROTTLE_SPEED = 2.5

# Lookup table for turns
_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]

def get_max_accel(v_ego):
  return np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)

def get_coast_accel(pitch):
  return np.sin(pitch) * -5.65 - 0.3  # fitted from data using xx/projects/allow_throttle/compute_coast_accel.py

def limit_accel_in_turns(v_ego, angle_steers, a_target, CP):
  """
  This function returns a limited long acceleration allowed, depending on the existing lateral acceleration
  this should avoid accelerating when losing the target in turns
  """
  # FIXME: This function to calculate lateral accel is incorrect and should use the VehicleModel
  # The lookup table for turns should also be updated if we do this
  a_total_max = np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
  a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))

  return [a_target[0], min(a_target[1], a_x_allowed)]


class LongitudinalPlanner(LongitudinalPlannerSP):
  def __init__(self, CP, CP_SP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    LongitudinalPlannerSP.__init__(self, self.CP, CP_SP, self.mpc)
    self.fcw = False
    self.dt = dt
    self.allow_throttle = True

    self.a_desired = init_a
    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.prev_accel_clip = [ACCEL_MIN, ACCEL_MAX]
    self.output_a_target = 0.0
    self.output_should_stop = False
    # BluePilot: the rate-limited e2e branch of the model-decel gate (never applied to the
    # mpc term); None whenever the gate is not participating, seeded at the live output on
    # first participation so entry is always continuous. _gate_e2e_prev feeds the model's own
    # per-frame fall through the down-limit (arbitration is paced, model braking is not).
    self._gate_branch_accel = None
    self._gate_e2e_prev = None

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)

  @staticmethod
  def parse_model(model_msg):
    if (len(model_msg.position.x) == ModelConstants.IDX_N and
      len(model_msg.velocity.x) == ModelConstants.IDX_N and
      len(model_msg.acceleration.x) == ModelConstants.IDX_N):
      x = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.position.x)
      v = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.velocity.x)
      a = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.acceleration.x)
      j = np.zeros(len(T_IDXS_MPC))
    else:
      x = np.zeros(len(T_IDXS_MPC))
      v = np.zeros(len(T_IDXS_MPC))
      a = np.zeros(len(T_IDXS_MPC))
      j = np.zeros(len(T_IDXS_MPC))
    if len(model_msg.meta.disengagePredictions.gasPressProbs) > 1:
      throttle_prob = model_msg.meta.disengagePredictions.gasPressProbs[1]
    else:
      throttle_prob = 1.0
    return x, v, a, j, throttle_prob

  def update(self, sm):
    LongitudinalPlannerSP.update(self, sm)

    if len(sm['carControl'].orientationNED) == 3:
      accel_coast = get_coast_accel(sm['carControl'].orientationNED[1])
    else:
      accel_coast = ACCEL_MAX

    v_ego = sm['carState'].vEgo
    v_cruise_kph = min(sm['carState'].vCruise, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS
    v_cruise_initialized = sm['carState'].vCruise != V_CRUISE_UNSET

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off
    force_slow_decel = sm['controlsState'].forceDecel

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
    # PCM cruise speed may be updated a few cycles later, check if initialized
    reset_state = reset_state or not v_cruise_initialized

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    accel_clip = [ACCEL_MIN, get_max_accel(v_ego)]
    steer_angle_without_offset = sm['carState'].steeringAngleDeg - sm['liveParameters'].angleOffsetDeg
    accel_clip = limit_accel_in_turns(v_ego, steer_angle_without_offset, accel_clip, self.CP)

    if reset_state:
      self.v_desired_filter.x = v_ego
      # Clip aEgo to cruise limits to prevent large accelerations when becoming active
      self.a_desired = np.clip(sm['carState'].aEgo, accel_clip[0], accel_clip[1])

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))
    _, _, _, _, throttle_prob = self.parse_model(sm['modelV2'])
    # Don't clip at low speeds since throttle_prob doesn't account for creep
    self.allow_throttle = throttle_prob > ALLOW_THROTTLE_THRESHOLD or v_ego <= MIN_ALLOW_THROTTLE_SPEED

    if not self.allow_throttle:
      clipped_accel_coast = max(accel_coast, accel_clip[0])
      clipped_accel_coast_interp = np.interp(v_ego, [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED*2], [accel_clip[1], clipped_accel_coast])
      accel_clip[1] = min(accel_clip[1], clipped_accel_coast_interp)

    # Get new v_cruise and a_desired from Smart Cruise Control and Speed Limit Assist.
    # BluePilot: snapshot last frame's PUBLISHED accel first - update_targets overwrites the
    # shared output_a_target attribute with the SP min-source's own value, so reading it later
    # would seed the gate branch from that instead of from what the car was actually commanded.
    prev_published_a_target = float(self.output_a_target)
    v_cruise, self.a_desired = LongitudinalPlannerSP.update_targets(self, sm, self.v_desired_filter.x, self.a_desired, v_cruise)

    if force_slow_decel:
      v_cruise = 0.0

    self.mpc.set_weights(prev_accel_constraint, personality=sm['selfdriveState'].personality)
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)
    self.mpc.update(sm['radarState'], v_cruise, personality=sm['selfdriveState'].personality)

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Interpolate 0.05 seconds and save as starting point for next iteration
    a_prev = self.a_desired
    self.a_desired = float(np.interp(self.dt, CONTROL_N_T_IDX, self.a_desired_trajectory))
    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.a_desired + a_prev) / 2.0

    action_t =  self.CP.longitudinalActuatorDelay + DT_MDL
    output_a_target_mpc, output_should_stop_mpc = get_accel_from_plan(self.v_desired_trajectory, self.a_desired_trajectory, CONTROL_N_T_IDX,
                                                                        action_t=action_t, vEgoStopping=self.CP.vEgoStopping)
    output_a_target_e2e = sm['modelV2'].action.desiredAcceleration
    output_should_stop_e2e = sm['modelV2'].action.shouldStop

    if self.is_e2e(sm):
      # BluePilot: with the model-decel gate, the e2e accel participates in the min() in
      # PROPORTION to the model's deceleration intent — a model plateauing at ~0.0 accel can
      # never hold the car below the set/limit target, while a model sinking toward the
      # engage setpoint blends in smoothly instead of snapping between "MPC accelerating"
      # and "full e2e deceleration" every time it hovers at the threshold (log-measured at
      # 10.9 flips/min with +0.7 m/s^2 half-second swings on release). shouldStop stays
      # OR'd regardless (it only fires near standstill and is the stop-hold latch).
      if self.model_decel_gate_enabled:
        model_v = sm['modelV2'].velocity.x
        model_end_v = model_v[len(model_v) - 1] if len(model_v) else v_ego
        self.decel_gate.update(output_a_target_e2e, bool(output_should_stop_e2e), model_end_v, v_ego)
        gate_w = self.decel_gate.weight
      else:
        # gate feature off: stock e2e arbitration — full participation, no branch limiting
        gate_w = None

      if gate_w is None:
        self._gate_branch_accel = None
        self.decel_gate.reset()
        output_a_target = min(output_a_target_e2e, output_a_target_mpc)
        if output_a_target < output_a_target_mpc:
          self.mpc.source = LongitudinalPlanSource.e2e
      elif gate_w > 0.0 or self._gate_branch_accel is not None:
        # the e2e branch, blended toward the MPC by how much of the ramp band the model's
        # intent has crossed: at the engage setpoint (weight 1) this is exactly the old
        # min(e2e, mpc); at the release threshold (weight 0) the branch equals the MPC and
        # the min() is a no-op. In between, the maximum acceleration lowers smoothly.
        #
        # The branch is then rate-limited in ACCEL space, both directions. Replaying the
        # blend without this showed why weight-space ramps alone are not enough: the blend's
        # d(weight) * (e2e - mpc) term steps the branch by the full spread times the weight
        # step, which on a wide spread is worse jerk than the binary gate ever produced.
        # Seeded at the live output on first participation (the old engage slew generalized:
        # entry is always continuous), falling at up to 2.5 m/s^3 (the engage rate this gate
        # has always used) and rising at up to 1.5 m/s^3 — so the release hand-back is a
        # comfort-jerk ramp instead of the old hold-then-snap, which log-measured at
        # +0.7 m/s^2 inside half a second, 10.9 gate flips/min. The mpc term is never
        # limited — lead braking stays untouched.
        #
        # The branch stays alive after the weight reaches zero until it has CONVERGED onto
        # the MPC: dropping it at weight 0 with the up-ramp still in flight would snap the
        # output by whatever gap remained (sim-measured at up to 17 m/s^3), exactly the
        # discontinuity this exists to remove.
        blend = gate_w * output_a_target_e2e + (1.0 - gate_w) * output_a_target_mpc
        if self._gate_branch_accel is None:
          self._gate_branch_accel = prev_published_a_target
        # The down-step may always be at least the weight-scaled movement of the MODEL's own
        # demand: the rate limit paces ARBITRATION transitions, never the model's braking
        # dynamics. Without this, a hazard dive arriving mid-engagement (where the old code
        # followed the model instantly) would be comfort-paced too — sim-measured 0.8 s
        # later to full braking, which is the wrong direction to spend comfort budget.
        e2e_fall = e2e_rise = 0.0
        if self._gate_e2e_prev is not None:
          e2e_fall = gate_w * max(0.0, self._gate_e2e_prev - output_a_target_e2e)
          e2e_rise = gate_w * max(0.0, output_a_target_e2e - self._gate_e2e_prev)
        down_step = max(2.5 * self.dt, e2e_fall)
        # the up-limit paces arbitration hand-backs only: a model recovering from its own
        # braking faster than 1.5 m/s^3 (green light, aborted brake) must not leave the
        # output braking below BOTH plan sources - review-measured 1.3 m/s of extra speed
        # lost per aborted brake and a ~1 s slower green-light go without this
        up_step = max(1.5 * self.dt, e2e_rise)
        self._gate_branch_accel = float(np.clip(blend, self._gate_branch_accel - down_step,
                                                self._gate_branch_accel + up_step))
        output_a_target = min(self._gate_branch_accel, output_a_target_mpc)
        if output_a_target < output_a_target_mpc:
          self.mpc.source = LongitudinalPlanSource.e2e
        if gate_w == 0.0 and self._gate_branch_accel >= output_a_target_mpc:
          self._gate_branch_accel = None
      else:
        self._gate_branch_accel = None
        output_a_target = output_a_target_mpc
      self._gate_e2e_prev = float(output_a_target_e2e)
      self.output_should_stop = output_should_stop_e2e or output_should_stop_mpc
    else:
      self._gate_branch_accel = None
      self._gate_e2e_prev = None
      self.decel_gate.reset()
      output_a_target = output_a_target_mpc
      self.output_should_stop = output_should_stop_mpc

    for idx in range(2):
      accel_clip[idx] = np.clip(accel_clip[idx], self.prev_accel_clip[idx] - 0.05, self.prev_accel_clip[idx] + 0.05)
    self.output_a_target = np.clip(output_a_target, accel_clip[0], accel_clip[1])
    self.prev_accel_clip = accel_clip

    # BluePilot: name this frame's binding constraint for the longitudinal target HUD.
    # The clip comparison must happen here — it is not derivable from published data.
    accel_clip_bound = bool(self.output_a_target != output_a_target)
    self.primary_limiter = classify_primary_limiter(bool(self.output_should_stop), bool(force_slow_decel),
                                                    accel_clip_bound, self.mpc.source, self.source)

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState', 'selfdriveState', 'radarState'])

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']
    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.status
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.aTarget = float(self.output_a_target)
    longitudinalPlan.shouldStop = bool(self.output_should_stop)
    longitudinalPlan.allowBrake = True
    longitudinalPlan.allowThrottle = bool(self.allow_throttle)

    pm.send('longitudinalPlan', plan_send)

    self.publish_longitudinal_plan_sp(sm, pm)
