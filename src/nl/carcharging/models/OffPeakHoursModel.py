from datetime import datetime
import logging

from marshmallow import fields, Schema
from marshmallow.fields import Boolean

from sqlalchemy import Column, Integer, Boolean, String, Time, orm, and_, or_, cast, Time, func
from nl.carcharging.models.Base import Base, DbSession


class OffPeakHoursModel(Base):
    __tablename__ = 'off_peak_hours'

    weekday_en = ['Monday', 'Tuesday', 'Wednesday', 'Thursday', 'Friday', 'Saturday', 'Sunday' ]
    weekday_nl = ['Maandag', 'Dinsdag', 'Woensdag', 'Donderdag', 'Vrijdag', 'Zaterdag', 'Zondag' ]

    id = Column(Integer, primary_key=True)
    weekday = Column(String(20))
    holiday_day = Column(Integer)
    holiday_month = Column(Integer)
    holiday_year = Column(Integer)
    recurring = Column(Boolean)
    description = Column(String(100))
    off_peak_start = Column(Time)
    off_peak_end = Column(Time)

    def __init__(self):
        self.logger = logging.getLogger('nl.carcharging.models.OffPeakHoursModel')

    # sqlalchemy calls __new__ not __init__ on reconstructing from database. Decorator to call this method
    @orm.reconstructor   
    def init_on_load(self):
        self.__init__()

    def set(self, data):
        for key in data:
            setattr(self, key, data.get(key))
        self.modified_at = datetime.datetime.now()

    def save(self):
        db_session = DbSession()
        try:
            db_session.add(self)
            db_session.commit()
        except Exception as e:
            db_session.rollback()
            self.logger.error("Could not save to {} table in database".format(self.__tablename__ ), exc_info=True)


    def delete(self):
        db_session = DbSession()
        try:
            db_session.delete(self)
            db_session.commit()
        except Exception as e:
            db_session.rollback()
            self.logger.error("Could not delete from {} table in database".format(self.__tablename__ ), exc_info=True)


    def weekdayToStr(self, weekday) -> str:
        return self.weekday_en[weekday % len(self.weekday_en)]


    # Timestamp of type datetime
    def is_off_peak_now(self) -> bool:
        return self.is_off_peak(timestamp=datetime.now())

    # Timestamp of type datetime
    def is_off_peak(self, timestamp) -> bool:
        self.logger.debug('is_off_peak()')
        if not isinstance(timestamp, datetime):
            self.logger.debug('is_off_peak() - timestamp is not of type datetime')
            return False
            
        db_session = DbSession()
        # Weekday?
        r = None
        try:
            r = db_session.query(OffPeakHoursModel) \
                          .filter(OffPeakHoursModel.weekday == self.weekdayToStr(timestamp.weekday())) \
                          .filter(OffPeakHoursModel.off_peak_start <= cast(timestamp, Time)) \
                          .filter(OffPeakHoursModel.off_peak_end >= cast(timestamp, Time))
        except Exception as e:
            # Nothing to roll back
            self.logger.error("Could not query from {} table in database".format(self.__tablename__ ), exc_info=True)
        if r is not None and self.get_count(r) > 0:
            self.logger.debug('is_off_peak(): DayOfWeek {} within off-peak'.format(str(timestamp.strftime("%d/%m/%Y, %H:%M:%S"))))
            return True

        # Is this a public holiday?
        r = None
        try:
            r = db_session.query(OffPeakHoursModel) \
                          .filter(
                              or_(
                                  and_(   # Specific holiday
                                      OffPeakHoursModel.holiday_day == int(timestamp.date().day),
                                      OffPeakHoursModel.holiday_month == int(timestamp.date().month),
                                      OffPeakHoursModel.holiday_year == int(timestamp.date().year)
                                  ),
                                  and_(   # Recurring holiday
                                      OffPeakHoursModel.holiday_day == int(timestamp.date().day),
                                      OffPeakHoursModel.holiday_month == int(timestamp.date().month),
                                      OffPeakHoursModel.recurring == True
                                  )
                              )
                              ) \
                          .filter(OffPeakHoursModel.off_peak_start <= cast(timestamp, Time)) \
                          .filter(OffPeakHoursModel.off_peak_end >= cast(timestamp, Time))
        except Exception as e:
            # Nothing to roll back
            self.logger.error("Could not query from {} table in database".format(self.__tablename__ ), exc_info=True)
        if r is not None and self.get_count(r) > 0:
            self.logger.debug('is_off_peak(): Holiday {} within off-peak'.format(str(timestamp.strftime("%d/%m/%Y, %H:%M:%S"))))
            return True

        self.logger.debug('is_off_peak(): {} not within off-peak'.format(str(timestamp.strftime("%d/%m/%Y, %H:%M:%S"))))
        return False


    def get_count(self, q):
        count = 0
        try:
            count_q = q.statement.with_only_columns([func.count()]).order_by(None)
            count = q.session.execute(count_q).scalar()
        except Exception as e:
            # Nothing to roll back
            self.logger.error("Could not query from {} table in database".format(self.__tablename__ ), exc_info=True)
        return count


    def __repr(self):
        return self.to_str()


    def to_str(self):
        return ({
                "id": str(self.id),
                "weekday": (str(self.weekday) if self.weekday is not None else None),
                "holiday": ((str(self.holiday_day) + '-' + str(self.holiday_month) + '-' + str(self.holiday_year)) \
                             if self.holiday_day is not None and self.holiday_month is not None and self.holiday_year is not None \
                             else None),
                "recurring": str(self.recurring),
                "description": str(self.description),
                "off_peak_start": (str(self.off_peak_start) if self.off_peak_start is not None else None),
                "off_peak_end": (str(self.off_peak_end) if self.off_peak_end is not None else None)
            }
        )


    def test(self):
        ohm = OffPeakHoursModel()
        # test None
        is_op = ohm.is_off_peak(None)
        # test other object (String)
        is_op = ohm.is_off_peak("testing")
        # test now
        ts = datetime.now()
        is_op = ohm.is_off_peak(ts)
        # test today at 22:15
        ts = ts.replace(hour=22, minute=15)
        is_op = ohm.is_off_peak(ts)
        # test today at 23:15
        ts = ts.replace(hour=23, minute=15)
        is_op = ohm.is_off_peak(ts)
        # test jan 1st 2019 as recurring date (is a Tuesday)
        ts = ts.replace(hour=14, minute=15)
        ts = ts.replace(day=1, month=1, year=2019)
        is_op = ohm.is_off_peak(ts)
        # test 1st x-mas day 2020 as recurring date (is a Friday)
        ts = ts.replace(day=25, month=12, year=2019)
        is_op = ohm.is_off_peak(ts)        


class ChargerConfigSchema(Schema):
    """
    ChargerConfigModel Schema
    """
    id = fields.Int(dump_only=True)
    weekday = fields.Str(dump_only=True)
    holiday_day = fields.Int(dump_only=True)
    holiday_month = fields.Int(dump_only=True)
    holiday_year = fields.Int(dump_only=True)
    recurring = fields.Bool(dump_only=True)
    description = fields.Str(dump_only=True)
    off_peak_start = fields.Time(dump_only=True)
    off_peak_end = fields.Time(dump_only=True)
