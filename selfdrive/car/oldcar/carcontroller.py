from common.numpy_fast import clip, interp
from selfdrive.boardd.boardd import can_list_to_can_capnp
from selfdrive.car.oldcar.oldcarcan import make_can_msg, create_steer_command, create_accel_command
from selfdrive.car.oldcar.values import ECU, STATIC_MSGS, NO_DSU_CAR
from selfdrive.can.packer import CANPacker

# Accel limits
ACCEL_HYST_GAP = 0.02  # don't change accel command for small oscilalitons within this value
ACCEL_MAX = 1.5  # 1.5 m/s2
ACCEL_MIN = -3.0 # 3   m/s2
ACCEL_SCALE = max(ACCEL_MAX, -ACCEL_MIN)

# Steer torque limits
STEER_MAX = 1500
STEER_DELTA_UP = 10       # 1.5s time to peak torque
STEER_DELTA_DOWN = 25     # always lower than 45 otherwise the Rav4 faults (Prius seems ok with 50)
STEER_ERROR_MAX = 350     # max delta between torque cmd and torque motor

# Steer angle limits (tested at the Crows Landing track and considered ok)
ANGLE_MAX_BP = [0., 5.]
ANGLE_MAX_V = [510., 300.]
ANGLE_DELTA_BP = [0., 5., 15.]
ANGLE_DELTA_V = [5., .8, .15]     # windup limit
ANGLE_DELTA_VU = [5., 3.5, 0.4]   # unwind limit

TARGET_IDS = [0x340, 0x341, 0x342, 0x343, 0x344, 0x345,
              0x363, 0x364, 0x365, 0x370, 0x371, 0x372,
              0x373, 0x374, 0x375, 0x380, 0x381, 0x382,
              0x383]


def accel_hysteresis(accel, accel_steady, enabled):

  # for small accel oscillations within ACCEL_HYST_GAP, don't change the accel command
  if not enabled:
    # send 0 when disabled, otherwise acc faults
    accel_steady = 0.
  elif accel > accel_steady + ACCEL_HYST_GAP:
    accel_steady = accel - ACCEL_HYST_GAP
  elif accel < accel_steady - ACCEL_HYST_GAP:
    accel_steady = accel + ACCEL_HYST_GAP
  accel = accel_steady

  return accel, accel_steady


class CarController(object):
  def __init__(self, dbc_name, car_fingerprint, enable_camera, enable_dsu, enable_apg):
    self.braking = False
    # redundant safety check with the board
    self.controls_allowed = True
    self.last_steer = 0
    self.last_angle = 0
    self.accel_steady = 0.
    self.angle_send = 0
    self.car_fingerprint = car_fingerprint
    self.alert_active = False
    self.last_standstill = False
    self.standstill_req = False
    self.angle_control = False

    self.steer_angle_enabled = False
    self.ipas_reset_counter = 0

    self.fake_ecus = set()
    if enable_camera: self.fake_ecus.add(ECU.CAM)
    if enable_dsu: self.fake_ecus.add(ECU.DSU)
    if enable_apg: self.fake_ecus.add(ECU.APGS)

    self.packer = CANPacker(dbc_name)

  def update(self, sendcan, enabled, CS, frame, actuators,
             pcm_cancel_cmd, hud_alert, audible_alert):

    # *** compute control surfaces ***

    #Leave this here, will someday use accel...
    
    # gas and brake
    apply_accel = actuators.gas - actuators.brake
    apply_accel, self.accel_steady = accel_hysteresis(apply_accel, self.accel_steady, enabled)
    apply_accel = clip(apply_accel * ACCEL_SCALE, ACCEL_MIN, ACCEL_MAX)

    # steer torque - leave minimum for steer torque incase it's needed
    apply_steer = int(round(actuators.steer * STEER_MAX))

    # steer angle - Currently using steer angle
    if enabled:
      apply_angle = actuators.steerAngle
      angle_lim = interp(CS.v_ego, ANGLE_MAX_BP, ANGLE_MAX_V)
      apply_angle = clip(apply_angle, -angle_lim, angle_lim)

      # windup slower
      if self.last_angle * apply_angle > 0. and abs(apply_angle) > abs(self.last_angle):
        angle_rate_lim = interp(CS.v_ego, ANGLE_DELTA_BP, ANGLE_DELTA_V)
      else:
        angle_rate_lim = interp(CS.v_ego, ANGLE_DELTA_BP, ANGLE_DELTA_VU)

      apply_angle = clip(apply_angle, self.last_angle - angle_rate_lim, self.last_angle + angle_rate_lim)
    else:
      apply_angle = CS.angle_steers

    #Disable if not enabled
    if not enabled:
      apply_angle = 0
      apply_steer_req = 0
    else:
      apply_steer_req = 1
      
    #If blinker on, apply_steer_req = 0 until released. This allows a smooth lane change
    if CS.left_blinker_on or CS.right_blinker_on or CS.brake_pressed:
      apply_steer_req = 0
    
    #Multply by 100 to allow 2 decmals sent over CAN. Arduino will divde by 100.
    angle_send = apply_angle * 100
    

    self.last_steer = apply_steer
    self.last_angle = apply_angle
    self.last_accel = apply_accel
    self.last_standstill = CS.standstill

    can_sends = []

    #Always send CAN steer message
    can_sends.append(create_steer_command(self.packer, angle_send, apply_steer_req, frame))

        
    # accel cmd comes from DSU, but we can spam can to cancel the system even if we are using lat only control
    if (frame % 3 == 0 and ECU.DSU in self.fake_ecus) or (pcm_cancel_cmd and ECU.CAM in self.fake_ecus):
      if ECU.DSU in self.fake_ecus:
        can_sends.append(create_accel_command(self.packer, apply_accel, pcm_cancel_cmd, self.standstill_req))
      else:
        can_sends.append(create_accel_command(self.packer, 0, pcm_cancel_cmd, False))



    sendcan.send(can_list_to_can_capnp(can_sends, msgtype='sendcan').to_bytes())
