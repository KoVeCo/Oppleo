import threading
import logging
import time

from nl.carcharging.config.WebAppConfig import WebAppConfig
from nl.carcharging.models.OffPeakHoursModel import OffPeakHoursModel
from nl.carcharging.models.ChargeSessionModel import ChargeSessionModel
from nl.carcharging.services.Evse import Evse
from nl.carcharging.utils.WebSocketUtil import WebSocketUtil


class PeakHoursMonitorThread(object):
    thread = None
    threadLock = None
    appSocketIO = None
    logger = None
    stop_event = None
    # Check EVSE status change every 2 seconds [seconds]
    changeEvseStatusCheckInterval = 2
    # Check Off Peak window every minute [seconds]
    offPeakWindowCheckInterval = 60

    sleepInterval = 0.25

    def __init__(self, \
                appSocketIO: appSocketIO \
                ):
        self.threadLock = threading.Lock()
        self.stop_event = threading.Event()
        self.logger = logging.getLogger('nl.carcharging.daemon.PeakHoursMonitorThread')
        self.appSocketIO = appSocketIO
        self.sleepInterval = min(self.changeEvseStatusCheckInterval, self.offPeakWindowCheckInterval) / 4


    def start(self):
        self.stop_event.clear()
        self.logger.debug('Launching Thread...')

        self.thread = threading.Thread(target=self.monitor, name='PeakHoursMonitorThread')
        self.thread.start()


    # PeakHoursMonitorThread
    def monitor(self):
        """
        Off peak hours
            If the EVSE is enabled during Peak hours, and the config is set to Off Peak, the EVSE is disabled untill the
            Off Peak hours start.
            Detecting Off Peak is database intensive. Checking if EVSE status has changed not.
                Quickly detect changed EVSE conditions - react when a session is started.
                Take time to see if Off Peak time period is entered

        Working
            Check for changed EVSE state every second, by quickly asking the EVSE Thread.
            Check for Off Peak every 5 minutes. 
            -   If a session is active, off peak, and the EVSE is disabled, (re-)enable the EVSE (and possibly wake 
                car for charging), oh, and reset any 'over-ride off peak for once' authorizations
            -   If Peak and reader is enabled, disable the EVSE, warn through push message (prowl)?
            Allow overriding off-peak for one period.
            Check only once per 1 or 5 minutes, to prevent database overload and bad rfid response 
        """
        changeEvseStatusCheckLastRun = 0
        offPeakWindowCheckLastRun = 0
        ohm = OffPeakHoursModel()
        evse = Evse()
        while not self.stop_event.is_set():

            try:
                """ 
                Off Peak Window Change check
                """
                if (time.time() *1000.0) > (offPeakWindowCheckLastRun + (self.offPeakWindowCheckInterval *1000.0)):
                    # Time to determine if it is Off Peak again
                    wasOffPeak = evse.isOffPeak
                    evse.isOffPeak = ohm.is_off_peak_now()
                    if (wasOffPeak != evse.isOffPeak):
                        # Send change notification
                        WebSocketUtil.emit(
                                event='off_peak_status_update', 
                                    id=WebAppConfig.ENERGY_DEVICE_ID,
                                    data={ 'isOffPeak': evse.isOffPeak,
                                        'offPeakEnabled': WebAppConfig.peakHoursOffPeakEnabled,
                                        'peakAllowOnePeriod': WebAppConfig.peakHoursAllowPeakOnePeriod
                                    },
                                    namespace='/charge_session',
                                    public=True
                                )
                    self.logger.debug('Off Peak Window Change check ... (wasOffPeak:{}, isOffPeak:{})'.format( \
                                    wasOffPeak, \
                                    evse.isOffPeak
                                    )
                                )
                    offPeakWindowCheckLastRun = time.time() *1000.0

                """ 
                EVSE Status Change check
                """
                if (time.time() *1000.0) > (changeEvseStatusCheckLastRun + (self.changeEvseStatusCheckInterval *1000.0)):
                    self.logger.debug('EVSE Status Change check ...')
                    # Time to check if the EVSE should be disabled in Peak or Enabled in Off Peak

                    # When to switch the EVSE off:
                    #    if the EVSE is enabled (off is already off), and
                    #    if it is Peak hours, and OffPeak enabled in settings, and not overridden for one session, 
                    #    switch the EVSE off untill the next Off Peak hours
                    if (evse.is_enabled() and \
                            ( not evse.isOffPeak and \
                            WebAppConfig.peakHoursOffPeakEnabled and \
                            not WebAppConfig.peakHoursAllowPeakOnePeriod  \
                            ) \
                    ):
                        self.logger.debug('Peak hours, EVSE ON and Off Peak enabled (not bypassed). Switching EVSE OFF')
                        # Switch the EVSE off untill Off Peak hours
                        evse.switch_off()  

                    """
                    # If the EVSE is not enabled, it is Peak hours, OffPeak is not enabled or
                    # OffPeak is enabled in settings, and overridden for one session, switch the EVSE on
                    if (not evse.isOffPeak and \
                        not evse.is_enabled() and \
                        ( not WebAppConfig.peakHoursOffPeakEnabled or \
                            ( WebAppConfig.peakHoursOffPeakEnabled and WebAppConfig.peakHoursAllowPeakOnePeriod) )):
                        self.logger.debug('Peak hours, EVSE ON and Off Peak enabled (not bypassed). Switching EVSE OFF')
                        # Switch the EVSE on
                        evse.switch_on()  
                    """

                    # When to switch the EVSE on:
                    #    if the EVSE is disabled (on is already on), and
                    #    if it is Off Peak hours, or 
                    #        if peakHours are not enabled, or
                    #        if PeakAllowed for once, 
                    #    and there is an active charge session (don't switch on without)
                    # Clear one-session override
                    if (not evse.is_enabled() and \
                        ( evse.isOffPeak or not WebAppConfig.peakHoursOffPeakEnabled or WebAppConfig.peakHoursAllowPeakOnePeriod) \
                    ):
                        # Only see if charge session exists in the database if the EVSE is enabled Off Peak
                        csm = ChargeSessionModel.get_open_charge_session_for_device(
                                                    WebAppConfig.ENERGY_DEVICE_ID
                                                    )
                        if csm is not None:
                            # Open charge session, enable the EVSE
                            self.logger.debug('Off Peak hours, EVSE OFF and Active charge session. Switching EVSE ON')
                            evse.switch_on()
                    if evse.isOffPeak:
                        with self.threadLock:
                            # Off peak now, reset the one session peak authorization
                            WebAppConfig.peakHoursAllowPeakOnePeriod = False

                    changeEvseStatusCheckLastRun = time.time() *1000.0
                    pass

                # Sleep for quite a while, and yield for other threads
                self.appSocketIO.sleep(self.sleepInterval)
            except Exception as e:
                self.logger.error("Exception tracking off peak hours", exc_info=True)


        self.logger.info("Stopping PeakHoursMonitorThread")


    def stop(self, block=False):
        self.logger.debug('Requested to stop')
        self.stop_event.set()


