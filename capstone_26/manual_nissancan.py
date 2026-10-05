#we will use this file to create manual our manual inputs rather
# than using nissancan.py, but we will use the same idea

#from opendbc.car.nissan.values import CAR
from opendbc.can import CANPacker
from opendbc.car.crc import CRC8J1850, mk_crc8_fun
from opendbc.car.nissan.values import CAR
#from opendbc.car.interfaces import CarControllerBase
#from opendbc.car.nissan import nissancan
#from opendbc.car.nissan.values import CAR, CarControllerParams
#from opendbc.car.common.filter_simple import FirstOrderFilter

# TODO: add this checksum to the CANPacker
nissan_checksum = mk_crc8_fun(CRC8J1850, init_crc=0x00, xor_out=0xFF)

#This is the main class for organizing CAN messages
class CarController_Manual():

  def __init__(self, dbc_name=None):    #Sends blank dbc until we send one

    #CANPacker will be initialized here when provided DBC name
    self.packer = None

    #Sends DBC into pre-made CAN packer
    if dbc_name is not None:
      self.packer = CANPacker(dbc_name)

     # can_sends = []
     # return can_sends

  # Hazard lights, only testing hazards for now
  def create_hazard_control(self, enabled):

      #Makes sure CANPacker is given DBC value
      if self.packer is None:
          raise RuntimeError("CANPacker not initialized")

      #Need these values from Cabana
      message_name = None
      bus = None
      #From cabana
      values = {}

      #Adding some safety checks
      if message_name is None or bus is None:
        raise NotImplementedError(
          "Hazard CAN message not identified"
      )

      #Now validated, create actual signals
      return self.packer.make_can_msg(message_name, bus, values)


