#!/usr/bin/env python

# evse_status based on read_PWM.py
# 2015-12-08
# Public Domain
import enum
import logging

import RPi.GPIO as GPIO

from nl.carcharging.config import Logger


class EvseReader:
    """
   A class to read PWM pulses and calculate their frequency
   and duty cycle.  The frequency is how often the pulse
   happens per second.  The duty cycle is the percentage of
   pulse high time per cycle.
   """

    def __init__(self, pi, gpio, weighting=0.0):
        """
      Instantiate with the Pi and gpio of the PWM signal
      to monitor.

      Optionally a weighting may be specified.  This is a number
      between 0 and 1 and indicates how much the old reading
      affects the new reading.  It defaults to 0 which means
      the old reading has no effect.  This may be used to
      smooth the data.
      """
        self.pi = pi
        self.gpio = gpio

        if weighting < 0.0:
            weighting = 0.0
        elif weighting > 0.99:
            weighting = 0.99

        self._new_weighting = 1.0 - weighting  # Weighting for new reading.
        self._old_weighting = weighting  # Weighting for old reading.

        self._high_tick = None
        self._pwm_frequency = None
        self._pwm_pulse_width_microseconds = None

        self._cb = pi.callback(gpio, pigpio.EITHER_EDGE, self._cbf)

    def _cbf(self, gpio, level, tick):

        # 1 = change to high (a rising edge)
        if level == 1:
            self._pwm_frequency = self._do_something_with_pulse(self._pwm_frequency, tick)
            self._high_tick = tick

        # 0 = change to low (a falling edge)
        elif level == 0:
            self._pwm_pulse_width_microseconds = self._do_something_with_pulse(self._pwm_pulse_width_microseconds, tick)

    def _do_something_with_pulse(self, old_value_of_interest, tick):
        new_value_of_interest = old_value_of_interest
        if self._high_tick is not None:
            t = pigpio.tickDiff(self._high_tick, tick)

            if old_value_of_interest is not None and self._old_weighting is not None and self._new_weighting is not None:
                new_value_of_interest = (self._old_weighting * old_value_of_interest) + (self._new_weighting * t)
            else:
                new_value_of_interest = t
        return new_value_of_interest

    def frequency(self):
        """
      Returns the PWM frequency.
      """
        if self._pwm_frequency is not None:
            return 1000000.0 / self._pwm_frequency
        else:
            return 0.0

    def pulse_width(self):
        """
      Returns the PWM pulse width in microseconds.
      """
        if self._pwm_pulse_width_microseconds is not None:
            return self._pwm_pulse_width_microseconds
        else:
            return 0.0

    def duty_cycle_percentage(self):
        """
      Returns the PWM duty cycle percentage.
      """
        if self._pwm_pulse_width_microseconds is not None and self._pwm_frequency is not None:
            return 100.0 * self._pwm_pulse_width_microseconds / self._pwm_frequency
        else:
            return 0.0

    def cancel(self):
        """
      Cancels the reader and releases resources.
      """
        self._cb.cancel()


class EvseState(enum.Enum):
    EVSE_STATE_UNKNOWN = 0  # Initial
    EVSE_STATE_INACTIVE = 1  # SmartEVSE State A: LED ON dimmed. Contactor OFF. Inactive
    EVSE_STATE_CONNECTED = 2  # SmartEVSE State B: LED ON full brightness, Car connected
    EVSE_STATE_CHARGING = 3  # SmartEVSE State C: charging (pulsing)
    EVSE_STATE_ERROR = 4  # SmartEVSE State ?: ERROR (quick pulse)


EVSE_MINLEVEL_STATE_CONNECTED = 8  # A dc lower than this indicates state A, higher state B

EVSE_TIME_TO_SWITCH_EDGES = 500  # Time a dcf can be steady while pulsing in ms
EVSE_TIME_TO_PULSE = 500  # Min time between rising edges to be pulsing. Faster is ERROR


def current_time_milliseconds():
    return time.time() * 1000


def is_current_measurement_over_edge_cycle(evse_measurement_milliseconds):
    '''
    Is time between evse_dcf_lastchange_timestamp_milliseconds and now greater than the time the evse needs
    to pulse.
    :param evse_measurement_milliseconds:
    :return: True means that if evse is pulsing, values should have changed by now
    '''
    return evse_measurement_milliseconds is not None and (
            (current_time_milliseconds() - evse_measurement_milliseconds) > EVSE_TIME_TO_SWITCH_EDGES)


def is_current_measurement_interval_normal_pulse(evse_measurement_milliseconds):
    '''
    Is time between evse_measurement_milliseconds the normal time the evse needs to pulse.
    :param evse_measurement_milliseconds:
    :return:
    '''
    return (evse_measurement_milliseconds is not None and (
            (current_time_milliseconds() - evse_measurement_milliseconds) >= EVSE_TIME_TO_PULSE))


if __name__ == "__main__":

    PROCESS_NAME = "EvseReader"
    Logger.init_log(PROCESS_NAME, "/tmp/%s.log" % PROCESS_NAME)
    logger = logging.getLogger("nl.carcharging.utils.EvseReader")

    import time
    import pigpio

    #   import read_PWM

    PWM_GPIO = 6  # 4
    RUN_TIME = 3600.0
    SAMPLE_TIME = 0.05  # .05 sec

    GPIO.setmode(GPIO.BCM)  # BCM / GIO mode
    GPIO.setup(6, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    pigpio_pi = pigpio.pi()

    evse_reader = EvseReader(pigpio_pi, PWM_GPIO)

    start = time.time()

    # -- Duty Cycle filtered
    evse_dcf_prev = None
    evse_changing = None
    evse_rising = None
    evse_rising_since = None
    evse_dcf_lastchange_timestamp_milliseconds = None

    evse_state = EvseState.EVSE_STATE_UNKNOWN  # active state INACTIVE | CONNECTED | CHARGING | ERROR

    """
     if the Duty Cycle is changing, assume it is charging
        detect the speed, is it too high, then it must be an error
     if the Duty Cycle is constant, check if it is 10  (CONNECTED), or lower (INACTIVE)

     possible required improvements
     - CONNECTED when 90 or higher?
     - when initially charging starts the freq can be measured as 22k and dc going from 0-2-0-1-1 (5x) -2-3-4 etc
       this triggers ERROR state. Place a filter on this.
   """

    logger.info(" Starting, state is {}".format(evse_state.name))
    while (time.time() - start) < RUN_TIME:

        time.sleep(SAMPLE_TIME)

        dc_percentage = evse_reader.duty_cycle_percentage()
        evse_dcf = round(dc_percentage / 10)  # duty cycle filtered [0-10]

        # first run
        if evse_dcf_prev is None:
            evse_dcf_prev = evse_dcf
            continue  # next iteration

        if evse_dcf == evse_dcf_prev:
            logger.debug("State seems stable")
            # possible state A (inactive), B (connected) or just a top/bottom  of a pulse
            # evse_dcf_lastchange_timestamp_milliseconds is not set if this while just started and values are stable.
            if evse_dcf_lastchange_timestamp_milliseconds is None:
                logger.debug("First run and evse value is stable.")
                evse_dcf_lastchange_timestamp_milliseconds = current_time_milliseconds()

            if is_current_measurement_over_edge_cycle(evse_dcf_lastchange_timestamp_milliseconds):
                logger.debug("Is stable for a while.")
                # this condition has been a while, must be state A (inactive) or B (connected0
                if evse_dcf >= EVSE_MINLEVEL_STATE_CONNECTED:
                    logger.debug("Evse is connected (charging?)")
                    # State B (Connected)
                    evse_rising = False
                    evse_rising_since = None
                    evse_state = EvseState.EVSE_STATE_CONNECTED
                else:
                    logger.debug("Evse is inactive (not charging?)")
                    # State A (Inactive)
                    evse_state = EvseState.EVSE_STATE_INACTIVE

        else:
            logger.debug("Value is changing")
            if evse_dcf > evse_dcf_prev:
                logger.debug("Value is rising")
                if evse_rising is None:  # starting up
                    logger.debug("Starting up")
                    evse_rising = True
                    evse_rising_since = current_time_milliseconds()
                else:
                    logger.debug("Switching from falling")
                    if not evse_rising:
                        if evse_rising_since is not None:
                            # switching to rising, quickly? (can this be ERROR?)
                            if not is_current_measurement_interval_normal_pulse(evse_rising_since):
                                logger.debug("ERROR situation, rising too fast")
                                evse_state = EvseState.EVSE_STATE_ERROR
                        evse_rising = True
                        evse_rising_since = current_time_milliseconds()
                    else:
                        logger.debug("Continue rising")
                        if is_current_measurement_interval_normal_pulse(evse_rising_since):
                            logger.debug("Evse is charging (value is rising)")
                            evse_state = EvseState.EVSE_STATE_CHARGING
            else:
                logger.debug("Value is falling")
                if is_current_measurement_interval_normal_pulse(evse_rising_since):
                    logger.debug("Evse is charging (value dropping)")
                    evse_state = EvseState.EVSE_STATE_CHARGING

                evse_rising = False
            # dcf has changed
            evse_dcf_lastchange_timestamp_milliseconds = current_time_milliseconds()
        # Remember current duty cycle for next run
        evse_dcf_prev = evse_dcf

    evse_reader.cancel()

    pigpio_pi.stop()
