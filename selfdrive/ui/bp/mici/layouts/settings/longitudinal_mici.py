"""BluePilot MICI: Longitudinal tuning panel — BP long bypass, coasting mode, downhill comp, Ford radar."""

from collections.abc import Callable

from openpilot.selfdrive.ui.bp.mici.widgets.button_bp import BigMultiParamToggleBP, BigParamControlBP
from openpilot.selfdrive.ui.bp.mici.widgets.floatbutton import BigParamFloatControl
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.widgets.scroller import NavScroller


class LongitudinalLayoutMici(NavScroller):
  def __init__(self, back_callback: Callable[[], None] | None = None):
    super().__init__()
    if back_callback is not None:
      self.set_back_callback(back_callback)

    self.disable_BP_long = BigParamControlBP("Bypass BP Longitudinal Control", "disable_BP_long_UI")
    self.coasting_mode = BigMultiParamToggleBP(
      "Coasting Mode", "FordPrefCoastingMode", ["Legacy", "Extended"],
    )
    self.disable_downhill_comp = BigParamControlBP("Disable Downhill Compensation", "disable_downhill_comp_UI")
    self.disable_ford_radar = BigParamControlBP("Disable Ford Radar (Vision-Only Leads)", "disable_ford_radar_UI")
    self.long_target_hud = BigParamControlBP("Longitudinal Controller Icons (Speed Sign)", "BPLongitudinalTargetHUD")
    self.model_decel_gate = BigParamControlBP("Model Decel Gate (E2E Brakes, ACC Accelerates)", "BPModelDecelGate")
    self.model_decel_gate_accel = BigParamFloatControl(
      "Decel Gate Engage Threshold (m/s²)", "BPModelDecelGateAccel", min=-5.00, max=0.00, step=0.01,
    )
    self.model_decel_gate_end_v = BigParamFloatControl(
      "Decel Gate Plan-End Margin (m/s)", "BPModelDecelGateEndV", min=-10.00, max=0.00, step=0.1,
    )
    self.limiter_window = BigParamFloatControl(
      "Controller Icons Window (s)", "BPLimiterWindow", min=0.0, max=5.0, step=0.1,
    )

    self._scroller.add_widgets([
      self.disable_BP_long,
      self.coasting_mode,
      self.disable_downhill_comp,
      self.disable_ford_radar,
      self.model_decel_gate,
      self.model_decel_gate_accel,
      self.model_decel_gate_end_v,
      self.long_target_hud,
      self.limiter_window,
    ])

    self._refresh_toggles = (
      ("disable_BP_long_UI", self.disable_BP_long),
      ("disable_downhill_comp_UI", self.disable_downhill_comp),
      ("disable_ford_radar_UI", self.disable_ford_radar),
      ("BPModelDecelGate", self.model_decel_gate),
      ("BPLongitudinalTargetHUD", self.long_target_hud),
    )

    ui_state.add_offroad_transition_callback(self._update_toggles)

  def show_event(self):
    super().show_event()
    self._update_toggles()

  def _update_toggles(self):
    ui_state.update_params()
    for key, item in self._refresh_toggles:
      item.set_checked(ui_state.params.get_bool(key))
    self.coasting_mode._load_value()
