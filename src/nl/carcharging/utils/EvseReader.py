# #!/usr/bin/env python
#
# # evse_status based on read_PWM.py
# # 2015-12-08
# # Public Domain
# import enum
# import logging
#
# import RPi.GPIO as GPIO
#
# from nl.carcharging.config import Logger
#
#
# class EvseReader:
#     """
#    A class to read PWM pulses and calculate their frequency
#    and duty cycle.  The frequency is how often the pulse
#    happens per second.  The duty cycle is the percentage of
#    pulse high time per cycle.
#    """
#
#     def __init__(self, pi, gpio, weighting=0.0):
#         """
#       Instantiate with the Pi and gpio of the PWM signal
#       to monitor.
#
#       Optionally a weighting may be specified.  This is a number
#       between 0 and 1 and indicates how much the old reading
#       affects the new reading.  It defaults to 0 which means
#       the old reading has no effect.  This may be used to
#       smooth the data.
#       """
#         self.pi = pi
#         self.gpio = gpio
#
#         if weighting < 0.0:
#             weighting = 0.0
#         elif weighting > 0.99:
#             weighting = 0.99
#
#         self._new_weighting = 1.0 - weighting  # Weighting for new reading.
#         self._old_weighting = weighting  # Weighting for old reading.
#
#         self._high_tick = None
#         self._pwm_frequency = None
#         self._pwm_pulse_width_microseconds = None
#
#         self._cb = pi.callback(gpio, pigpio.EITHER_EDGE, self._cbf)
#
#     def _cbf(self, gpio, level, tick):
#
#         # 1 = change to high (a rising edge)
#         if level == 1:
#             self._pwm_frequency = self._do_something_with_pulse(self._pwm_frequency, tick)
#             self._high_tick = tick
#
#         # 0 = change to low (a falling edge)
#         elif level == 0:
#             self._pwm_pulse_width_microseconds = self._do_something_with_pulse(self._pwm_pulse_width_microseconds, tick)
#
#     def _do_something_with_pulse(self, old_value_of_interest, tick):
#         new_value_of_interest = old_value_of_interest
#         if self._high_tick is not None:
#             t = pigpio.tickDiff(self._high_tick, tick)
#
#             if old_value_of_interest is not None and self._old_weighting is not None and self._new_weighting is not None:
#                 new_value_of_interest = (self._old_weighting * old_value_of_interest) + (self._new_weighting * t)
#             else:
#                 new_value_of_interest = t
#         return new_value_of_interest
#
#     def frequency(self):
#         """
#       Returns the PWM frequency.
#       """
#         if self._pwm_frequency is not None:
#             return 1000000.0 / self._pwm_frequency
#         else:
#             return 0.0
#
#     def pulse_width(self):
#         """
#       Returns the PWM pulse width in microseconds.
#       """
#         if self._pwm_pulse_width_microseconds is not None:
#             return self._pwm_pulse_width_microseconds
#         else:
#             return 0.0
#
#     def duty_cycle_percentage(self):
#         """
#       Returns the PWM duty cycle percentage.
#       """
#         if self._pwm_pulse_width_microseconds is not None and self._pwm_frequency is not None:
#             return 100.0 * self._pwm_pulse_width_microseconds / self._pwm_frequency
#         else:
#             return 0.0
#
#     def evse_value(self):
#         '''
#         Returns the duty_cycle_percentage but 1-100 is mapped to 1-10
#         :return: duty-cycle_percentage/10 (int)
#         '''
#         return round(self.duty_cycle_percentage() / 10)
#
#     def cancel(self):
#         """
#       Cancels the reader and releases resources.
#       """
#         self._cb.cancel()
#
#
# class EvseDirection(enum.Enum):
#     UP = 1
#     DOWN = -1
#     NONE = 0
#
#
# class EvseState(enum.Enum):
#     EVSE_STATE_UNKNOWN = 0  # Initial
#     EVSE_STATE_INACTIVE = 1  # SmartEVSE State A: LED ON dimmed. Contactor OFF. Inactive
#     EVSE_STATE_CONNECTED = 2  # SmartEVSE State B: LED ON full brightness, Car connected
#     EVSE_STATE_CHARGING = 3  # SmartEVSE State C: charging (pulsing)
#     EVSE_STATE_ERROR = 4  # SmartEVSE State ?: ERROR (quick pulse)
#
#
# EVSE_MINLEVEL_STATE_CONNECTED = 8  # A dc lower than this indicates state A, higher state B
#
# EVSE_TIME_TO_PULSE = 500  # Min time between rising edges to be pulsing. Faster is ERROR
#
#
# def current_time_milliseconds():
#     return time.time() * 1000
#
#
# def determine_evse_direction(delta):
#     delta_int = round(delta)
#
#     direction = EvseDirection.NONE
#     if delta_int > 0:
#         direction = EvseDirection.UP
#     elif delta_int < 0:
#         direction = EvseDirection.DOWN
#
#     return direction
#
#
# def is_current_measurement_interval_normal_pulse(evse_measurement_milliseconds):
#     '''
#     Is duration since evse_measurement_milliseconds the normal time the evse needs to pulse.
#     :param evse_measurement_milliseconds:
#     :return:
#     '''
#     delta = current_time_milliseconds() - evse_measurement_milliseconds
#     logger.debug('Normal pulse cycle? interval of change is calculated: %f' % delta)
#     return evse_measurement_milliseconds is not None and (delta >= EVSE_TIME_TO_PULSE)
#
#
# def is_pulse_direction_changed(direction_current, direction_previous):
#     return direction_current != direction_previous
#
#
# if __name__ == "__main__":
#
#     PROCESS_NAME = "EvseReader"
#     Logger.init_log(PROCESS_NAME, "/tmp/%s.log" % PROCESS_NAME)
#     logger = logging.getLogger("nl.carcharging.utils.EvseReader")
#
#     import time
#     import pigpio
#
#     #   import read_PWM
#
#     PWM_GPIO = 6  # 4
#     RUN_TIME = 3600.0
#     SAMPLE_TIME = 0.05  # .05 sec
#
#     GPIO.setmode(GPIO.BCM)  # BCM / GIO mode
#     GPIO.setup(6, GPIO.IN, pull_up_down=GPIO.PUD_UP)
#
#     pigpio_pi = pigpio.pi()
#
#     evse_reader = EvseReader(pigpio_pi, PWM_GPIO)
#
#     start = time.time()
#
#     evse_state = EvseState.EVSE_STATE_UNKNOWN  # active state INACTIVE | CONNECTED | CHARGING | ERROR
#
#     """
#      if the Duty Cycle is changing, assume it is charging
#         detect the speed, is it too high, then it must be an error
#      if the Duty Cycle is constant, check if it is 10  (CONNECTED), or lower (INACTIVE)
#
#      possible required improvements
#      - CONNECTED when 90 or higher?
#      - when initially charging starts the freq can be measured as 22k and dc going from 0-2-0-1-1 (5x) -2-3-4 etc
#        this triggers ERROR state. Place a filter on this.
#    """
#
#     # Overall direction of the measured pulse. If current measure is equal to previous measure (EvseDirection.NONE)
#     # the overall direction stays the direction before the direction became NONE. So when the measured value changes
#     # we can see if it continues the UP or DOWN or switches. Duration of the direction (evse_direction_change_moment)
#     # is input for us to determine if evse is charging or in error (pulses faster).
#     evse_direction_overall = None
#     # Direction of the pulse when we saw the direction of pulse switched (the new direction)
#     # so together with the evse_direction_overall (current direction, ignoring EvseDirection.NONE measures)
#     # we can see if the direction switched (and hence means evse is charging or in error).
#     evse_direction_overall_previous = None
#     # Current direction of pulse. UP, DOWN or NONE.
#     evse_direction_current = None
#     # Contains the moment the direction_overall changes from UP to DOWN or vv.
#     evse_direction_change_moment = current_time_milliseconds()
#     # Timestamp since the evse values didn't change anymore (if applicable, otherwise None)
#     evse_stable_since = None
#
#     # Current and previous evse measure. To know if direction of pulse is UP, DOWN or NONE
#     evse_dcf = None
#     evse_dcf_prev = None
#
#     logger.info(" Starting, state is {}".format(evse_state.name))
#     while (time.time() - start) < RUN_TIME:
#
#         time.sleep(SAMPLE_TIME)
#
#         evse_dcf = evse_reader.evse_value()
#
#         # First run?
#         if evse_dcf_prev is None:
#             evse_dcf_prev = evse_dcf
#             continue  # next iteration
#
#         evse_direction_current = determine_evse_direction(evse_dcf - evse_dcf_prev)
#         if evse_direction_current != EvseDirection.NONE:
#             evse_direction_overall = evse_direction_current
#
#         logger.debug('evse_current and prev %f vs %f' % (evse_dcf, evse_dcf_prev))
#         if evse_direction_current == EvseDirection.NONE:
#             logger.debug(
#                 'Direction is neutral. Overall direction %s.' % evse_direction_overall.name if evse_direction_overall else '<null>')
#             if evse_stable_since is None:
#                 evse_stable_since = current_time_milliseconds()
#
#             if is_current_measurement_interval_normal_pulse(evse_stable_since):
#                 logger.debug('In the time-span a pulse would change direction, the evse value did not change')
#                 if evse_dcf >= EVSE_MINLEVEL_STATE_CONNECTED:
#                     logger.debug("Evse is connected (not charging)")
#                     evse_state = EvseState.EVSE_STATE_CONNECTED
#                 else:
#                     logger.debug("Evse is inactive (not charging)")
#                     # State A (Inactive)
#                     evse_state = EvseState.EVSE_STATE_INACTIVE
#         else:
#             # Evse measure changed
#             evse_stable_since = None
#             if is_pulse_direction_changed(evse_direction_overall, evse_direction_overall_previous):
#                 logger.debug(
#                     'Direction of evse dutycycle changed. Current direction overall: %s' % evse_direction_overall.name)
#                 if is_current_measurement_interval_normal_pulse(evse_direction_change_moment):
#                     evse_state = EvseState.EVSE_STATE_CHARGING
#                 else:
#                     # Too fast, means error.
#                     evse_state = EvseState.EVSE_STATE_ERROR
#                 evse_direction_overall_previous = evse_direction_overall
#                 evse_direction_change_moment = current_time_milliseconds()
#
#         logger.debug("Current evse_state %s" % evse_state.name)
#         # Remember current evse direction for next run
#         evse_direction_previous = evse_direction_overall
#         # Remember current duty cycle for next run
#         evse_dcf_prev = evse_dcf
#
#     evse_reader.cancel()
#
#     pigpio_pi.stop()
