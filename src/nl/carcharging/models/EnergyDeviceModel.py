from marshmallow import fields, Schema

from nl.carcharging.models.base import Base, DbSession
from . import db


class EnergyDeviceModel(Base):
    """
    EnergyDevice Model
    """

    # table name
    __tablename__ = 'energy_device'

    energy_device_id = db.Column(db.String(100), primary_key=True)
    port_name = db.Column(db.String(100))
    slave_address = db.Column(db.Integer)

    def __init__(self, data):
        self.energy_device_id = data.get('energy_device_id')

    def save(self):
        session = DbSession()
        session.add(self)
        session.commit()

    def delete(self):
        session = DbSession()
        session.delete(self)
        session.commit()

    @staticmethod
    def get_all():
        session = DbSession()
        return session.query(EnergyDeviceModel).all()

    @staticmethod
    def get_one(energy_device_id):
        session = DbSession()
        return session.query(EnergyDeviceModel)\
            .filter(EnergyDeviceModel.energy_device_id == energy_device_id).first()

    def __repr(self):
        return '<id {}>'.format(self.id)

class EnergyDeviceSchema(Schema):
    """
    Energy Device Schema
    """
    energy_device_id = fields.Str(required=True)
    port_name = fields.Str(dump_only=True)
    slave_address = fields.Int(dump_only=True)
