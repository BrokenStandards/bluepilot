"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from enum import IntEnum

from openpilot.selfdrive.ui.sunnypilot.layouts.settings.cruise_sub_layouts.speed_limit_settings import SpeedLimitSettingsLayout
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.sunnypilot.widgets.list_view import toggle_item_sp, option_item_sp, simple_button_item_sp
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.scroller_tici import Scroller


class PanelType(IntEnum):
  CRUISE = 0
  SLA = 1


ICBM_DESC = tr_noop("When enabled, sunnypilot will attempt to manage the built-in cruise control buttons " +
                    "by emulating button presses for limited longitudinal control.")
# BluePilot: ICBM also runs under alpha long, walking the cluster set speed to the confirmed
# speed limit plus a configurable margin.
ICBM_ALPHA_LONG_DESC = tr_noop("With openpilot longitudinal active, ICBM keeps the cluster set speed at the confirmed " +
                               "speed limit plus the margin below, so the vehicle never limits acceleration near the target.")
ICBM_OFFSET_DESC = tr_noop("How far above the desired speed ICBM keeps the cluster set speed under openpilot longitudinal. " +
                           "Vehicle dependent: must clear the cruise module's near-set-speed acceleration limit.")
ICMB_UNAVAILABLE = tr_noop("Intelligent Cruise Button Management is currently unavailable on this platform.")

ACC_ENABLED_DESCRIPTION = tr_noop("Enable custom Short & Long press increments for cruise speed increase/decrease.")
ACC_NOLONG_DESCRIPTION = tr_noop("This feature can only be used with sunnypilot longitudinal control enabled.")
ACC_PCMCRUISE_DISABLED_DESCRIPTION = tr_noop("This feature is not supported on this platform due to vehicle limitations.")
ONROAD_ONLY_DESCRIPTION = tr_noop("Start the vehicle to check vehicle compatibility.")


class CruiseLayout(Widget):
  def __init__(self):
    super().__init__()
    self._current_panel = PanelType.CRUISE
    self._speed_limit_layout = SpeedLimitSettingsLayout(lambda: self._set_current_panel(PanelType.CRUISE))

    items = self._initialize_items()
    self._scroller = Scroller(items, line_separator=True, spacing=0)

  def _initialize_items(self):

    self.icbm_toggle = toggle_item_sp(
      title=tr("Intelligent Cruise Button Management (ICBM) (Alpha)"),
      description="",
      param="IntelligentCruiseButtonManagement")

    self.icbm_offset_item = option_item_sp(
      title=tr("Alpha Long ICBM Set Speed Margin"),
      param="AlphaLongIcbmOffset",
      min_value=1, max_value=15, value_change_step=1,
      description=tr(ICBM_OFFSET_DESC),
      label_callback=self._get_icbm_offset_label,
      inline=True)

    self.scc_v_toggle = toggle_item_sp(
      title=tr("Smart Cruise Control - Vision"),
      description=tr("Use vision path predictions to estimate the appropriate speed to drive through turns ahead."),
      param="SmartCruiseControlVision")

    self.scc_m_toggle = toggle_item_sp(
      title=tr("Smart Cruise Control - Map"),
      description=tr("Use map data to estimate the appropriate speed to drive through turns ahead."),
      param="SmartCruiseControlMap")

    # BluePilot: mapd_bp reads this at runtime (no longitudinal dependency), so the
    # toggle stays enabled regardless of has_long/has_icbm below.
    self.visual_routing_assist_toggle = toggle_item_sp(
      title=tr("Visual Routing Assistance"),
      description=tr("Uses the car's actual driven path in addition to GPS to match the road on ramps and forks sooner."),
      param="VisualRoutingAssist")
    # End BluePilot

    self.custom_acc_toggle = toggle_item_sp(
      title=tr("Custom ACC Speed Increments"),
      description="",
      param="CustomAccIncrementsEnabled",
      callback=self._on_custom_acc_toggle)

    self.custom_acc_short_increment = option_item_sp(
      title=tr("Short Press Increment"),
      param="CustomAccShortPressIncrement",
      min_value=1, max_value=10, value_change_step=1,
      inline=True)

    self.custom_acc_long_increment = option_item_sp(
      title=tr("Long Press Increment"),
      param="CustomAccLongPressIncrement",
      value_map={1: 1, 2: 5, 3: 10},
      min_value=1, max_value=3, value_change_step=1,
      inline=True)

    self.sla_settings_button = simple_button_item_sp(
      button_text=lambda: tr("Speed Limit"),
      button_width=800,
      callback=lambda: self._set_current_panel(PanelType.SLA)
    )

    self.dec_toggle = toggle_item_sp(
      title=tr("Enable Dynamic Experimental Control"),
      description=tr("Enable toggle to allow the model to determine when to use sunnypilot ACC or sunnypilot End to End Longitudinal."),
      param="DynamicExperimentalControl")

    # BluePilot: per-frame arbitration — takes precedence over DEC when both are enabled
    self.model_decel_gate_toggle = toggle_item_sp(
      title=tr("Model Decel Gate (E2E Brakes, ACC Accelerates)"),
      description=tr("In Experimental Mode, the driving model may only slow the car (early braking for lights, " +
                     "hazards and stops); ACC owns acceleration and cruise, so the model can no longer hold the car " +
                     "below the set speed or speed limit. Takes precedence over Dynamic Experimental Control."),
      param="BPModelDecelGate")

    items = [
      self.icbm_toggle,
      self.icbm_offset_item,
      self.dec_toggle,
      self.model_decel_gate_toggle,
      self.scc_v_toggle,
      self.scc_m_toggle,
      # BluePilot: Visual Routing Assistance toggle
      self.visual_routing_assist_toggle,
      # End BluePilot
      self.custom_acc_toggle,
      self.custom_acc_short_increment,
      self.custom_acc_long_increment,
      self.sla_settings_button,
    ]
    return items

  def _render(self, rect):
    if self._current_panel == PanelType.SLA:
      self._speed_limit_layout.render(rect)
    else:
      self._scroller.render(rect)

  def show_event(self):
    self._set_current_panel(PanelType.CRUISE)
    self._scroller.show_event()
    self.icbm_toggle.show_description(True)
    self.custom_acc_toggle.show_description(True)

  def _set_current_panel(self, panel: PanelType):
    self._current_panel = panel
    if panel == PanelType.SLA:
      self._speed_limit_layout.show_event()

  @staticmethod
  def _get_icbm_offset_label(value):
    unit = tr("km/h") if ui_state.is_metric else tr("mph")
    return f"+{value} {unit}"

  def _update_state(self):
    super()._update_state()

    if ui_state.CP is not None and ui_state.CP_SP is not None:
      has_icbm = ui_state.has_icbm
      has_long = ui_state.has_longitudinal_control

      # BluePilot: ICBM is allowed with or without alpha long — under alpha long it walks the
      # cluster to the confirmed limit + margin instead of owning the set speed.
      if ui_state.CP_SP.intelligentCruiseButtonManagementAvailable:
        self.icbm_toggle.action_item.set_enabled(ui_state.is_offroad())
        new_desc = tr(ICBM_DESC) if not has_long else tr(ICBM_DESC) + "\n\n" + tr(ICBM_ALPHA_LONG_DESC)
        if self.icbm_toggle.description != new_desc:
          self.icbm_toggle.set_description(new_desc)
      else:
        ui_state.params.remove("IntelligentCruiseButtonManagement")
        self.icbm_toggle.action_item.set_enabled(False)

        new_desc = "<b>" + tr(ICMB_UNAVAILABLE) + "</b>\n\n" + tr(ICBM_DESC)
        if self.icbm_toggle.description != new_desc:
          self.icbm_toggle.set_description(new_desc)
          self.icbm_toggle.show_description(True)

      # margin spinner only matters for alpha-long ICBM
      self.icbm_offset_item.action_item.set_enabled(has_icbm and has_long and ui_state.is_offroad())

      if has_long or has_icbm:
        self.custom_acc_toggle.action_item.set_enabled(((has_long and not ui_state.CP.pcmCruise) or has_icbm) and ui_state.is_offroad())
        self.dec_toggle.action_item.set_enabled(has_long)
        self.model_decel_gate_toggle.action_item.set_enabled(has_long)
        self.scc_v_toggle.action_item.set_enabled(True)
        self.scc_m_toggle.action_item.set_enabled(True)
      else:
        ui_state.params.remove("CustomAccIncrementsEnabled")
        ui_state.params.remove("DynamicExperimentalControl")
        ui_state.params.remove("BPModelDecelGate")
        ui_state.params.remove("SmartCruiseControlVision")
        ui_state.params.remove("SmartCruiseControlMap")
        self.custom_acc_toggle.action_item.set_enabled(False)
        self.dec_toggle.action_item.set_enabled(False)
        self.model_decel_gate_toggle.action_item.set_enabled(False)
        self.scc_v_toggle.action_item.set_enabled(False)
        self.scc_m_toggle.action_item.set_enabled(False)

    else:
      has_icbm = has_long = False
      self.icbm_toggle.action_item.set_enabled(False)
      self.icbm_toggle.set_description(tr(ONROAD_ONLY_DESCRIPTION))

    show_custom_acc_desc = False

    if ui_state.is_offroad():
      new_custom_acc_desc = tr(ONROAD_ONLY_DESCRIPTION)
      show_custom_acc_desc = True
    else:
      if has_long or has_icbm:
        if has_long and ui_state.CP.pcmCruise:
          new_custom_acc_desc = tr(ACC_PCMCRUISE_DISABLED_DESCRIPTION)
          show_custom_acc_desc = True
        else:
          new_custom_acc_desc = tr(ACC_ENABLED_DESCRIPTION)
      else:
        new_custom_acc_desc = tr(ACC_NOLONG_DESCRIPTION)
        show_custom_acc_desc = True
        self.custom_acc_toggle.action_item.set_state(False)

    if self.custom_acc_toggle.description != new_custom_acc_desc:
      self.custom_acc_toggle.set_description(new_custom_acc_desc)
      if show_custom_acc_desc:
        self.custom_acc_toggle.show_description(True)

    self._on_custom_acc_toggle(self.custom_acc_toggle.action_item.get_state())

  def _on_custom_acc_toggle(self, state):
    self.custom_acc_short_increment.set_visible(state)
    self.custom_acc_long_increment.set_visible(state)
    self.custom_acc_short_increment.action_item.set_enabled(self.custom_acc_toggle.action_item.enabled)
    self.custom_acc_long_increment.action_item.set_enabled(self.custom_acc_toggle.action_item.enabled)
