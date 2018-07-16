import struct
from ctypes import create_string_buffer


def add_tesla_checksum(msg_id,msg):
 """Calculates the checksum for the data part of the Tesla message"""
 checksum = ((msg_id) & 0xFF) + ((msg_id >> 8) & 0xFF)
 for i in range(0,len(msg),1):
  checksum = (checksum + ord(msg[i])) & 0xFF
 return checksum


def create_steering_control(enabled, apply_steer, idx):
 """Creates a CAN message for the Tesla DBC DAS_steeringControl."""
 msg_id = 0x488
 msg_len = 4
 msg = create_string_buffer(msg_len)
 if enabled == False:
  steering_type = 0
 else:
  steering_type = 1
 type_counter = steering_type << 6
 type_counter += idx
 struct.pack_into('!hB', msg, 0,  apply_steer, type_counter)
 struct.pack_into('B', msg, msg_len-1, add_tesla_checksum(msg_id,msg))
 return [msg_id, 0, msg.raw, 2]


def create_epb_enable_signal(idx):
  """Creates a CAN message to simulate EPB enable message"""
  msg_id = 0x214
  msg_len = 3
  msg = create_string_buffer(msg_len)
  struct.pack_into('BB', msg, 0,  1, idx)
  struct.pack_into('B', msg, msg_len-1, add_tesla_checksum(msg_id,msg))
  return [msg_id, 0, msg.raw, 2]
  
def create_cruise_adjust_msg(spdCtrlLvr_stat, idx):
  """Creates a CAN message from the cruise control stalk.
  
  Simluates pressing the cruise control stalk (STW_ACTN_RQ.SpdCtrlLvr_Stat
   
  It is probably best not to flood these messages so that the real
  stalk works normally.
  
  Args:
    spdCtrlLvr_stat: Int value of dbc entry STW_ACTN_RQ.SpdCtrlLvr_Stat
  """
  msg_id = 0x45  # 69 in hex, STW_ACTN_RQ
  msg_len = 8
  msg = create_string_buffer(msg_len)
  b0 = ( spdCtrlLvr_stat << 2 ) + 2 # 2 is to set the VSL_Enbl_Rq as 1
  struct.pack_into('B', msg, 0,  b0)
  # set DTR_Dist_Rq, 8 bits of ones.
  struct.pack_into('B', msg, 1,  255)
  struct.pack_into('BBBBB', msg, 2, 0, 0, 0, 0, idx)
  struct.pack_into('B', msg, msg_len-1, add_tesla_checksum(msg_id,msg))
  return [msg_id, 0, msg.raw, 0]  

def _cruise_stalk_checksum(spdCtrlLvr_stat, idx): 
  # map of observed crcs, modeled as a nested dict of
  # spdCtrlLvr_stat:message_count:observed_crc
  # TODO: Replace this hard-coded map with an actual checksum
  #       calculation.
  crcs = {
    4: {  # UP_2ND
      0: 180,
      1: 121,
      2: 51,
      3: 254,
      4: 167,
      5: 106,
      6: 32,
      7: 237,
      8: 146,
      9: 95,
      10: 21,
      11: 216,
      12: 129,
      13: 76,
      14: 6,
      15: 203
    },
    8: {  # DN_2ND
      0: 15,
      1: 194,
      2: 136,
      3: 69,
      4: 28,
      5: 209,
      6: 155,
      7: 86,
      8: 41,
      9: 228,
      10: 174,
      11: 99,
      12: 58,
      13: 247,
      14: 189,
      15: 112
    },
    16: {  # UP_1ST
      0: 100,
      1: 169,
      #2: TODO,
      3: 46,
      4: 119,
      #5: TODO,
      #6: TODO,
      7: 61,
      8: 66,
      9: 143,
      10: 197,
      #11: TODO,
      #12: TODO,
      13: 156,
      14: 214,
      15: 27
    },
    32: {  # DN_1ST
      0: 178,
      1: 127,
      2: 53,
      #3: TODO,
      #4: TODO,
      5: 108,
      6: 38,
      7: 235,
      8: 148,
      9: 89,
      10: 19,
      11: 222,
      12: 135,
      13: 74,
      14: 0,
      15: 205
    }
  }
  # try looking up the CRC in a list of observed values.
  if spdCtrlLvr_stat in crcs:
    position_crcs = crcs[spdCtrlLvr_stat]
    if idx in position_crcs:
      return position_crcs[idx]
  return 0
