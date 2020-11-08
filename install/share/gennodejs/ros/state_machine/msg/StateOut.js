// Auto-generated. Do not edit!

// (in-package state_machine.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class StateOut {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.Diagnostics = null;
      this.PodInfo = null;
      this.HMSCheck = null;
      this.OperationMode = null;
      this.PrevState = null;
      this.CurrState = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('Diagnostics')) {
        this.Diagnostics = initObj.Diagnostics
      }
      else {
        this.Diagnostics = '';
      }
      if (initObj.hasOwnProperty('PodInfo')) {
        this.PodInfo = initObj.PodInfo
      }
      else {
        this.PodInfo = 0;
      }
      if (initObj.hasOwnProperty('HMSCheck')) {
        this.HMSCheck = initObj.HMSCheck
      }
      else {
        this.HMSCheck = 0;
      }
      if (initObj.hasOwnProperty('OperationMode')) {
        this.OperationMode = initObj.OperationMode
      }
      else {
        this.OperationMode = 0;
      }
      if (initObj.hasOwnProperty('PrevState')) {
        this.PrevState = initObj.PrevState
      }
      else {
        this.PrevState = 0;
      }
      if (initObj.hasOwnProperty('CurrState')) {
        this.CurrState = initObj.CurrState
      }
      else {
        this.CurrState = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type StateOut
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [Diagnostics]
    bufferOffset = _serializer.string(obj.Diagnostics, buffer, bufferOffset);
    // Serialize message field [PodInfo]
    bufferOffset = _serializer.uint8(obj.PodInfo, buffer, bufferOffset);
    // Serialize message field [HMSCheck]
    bufferOffset = _serializer.uint8(obj.HMSCheck, buffer, bufferOffset);
    // Serialize message field [OperationMode]
    bufferOffset = _serializer.uint8(obj.OperationMode, buffer, bufferOffset);
    // Serialize message field [PrevState]
    bufferOffset = _serializer.uint8(obj.PrevState, buffer, bufferOffset);
    // Serialize message field [CurrState]
    bufferOffset = _serializer.uint8(obj.CurrState, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type StateOut
    let len;
    let data = new StateOut(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [Diagnostics]
    data.Diagnostics = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [PodInfo]
    data.PodInfo = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [HMSCheck]
    data.HMSCheck = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [OperationMode]
    data.OperationMode = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [PrevState]
    data.PrevState = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [CurrState]
    data.CurrState = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.Diagnostics.length;
    return length + 9;
  }

  static datatype() {
    // Returns string type for a message object
    return 'state_machine/StateOut';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '709ee385b8a9e470a0649c6afc4d01c8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 State_Idle=0
    uint8 State_P2P=1
    uint8 State_Identify=2
    uint8 State_Approach=3
    uint8 State_Verify=4
    uint8 State_Retrace=5
    uint8 State_Lock=6
    uint8 State_Unlock=7
    uint8 State_EHS= 8
    uint8 OperationMode_Pickup = 1
    uint8 OperationMode_DropOff = 2
    Header header 
    string Diagnostics
    uint8 PodInfo
    uint8 HMSCheck
    uint8 OperationMode
    uint8 PrevState
    uint8 CurrState
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new StateOut(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.Diagnostics !== undefined) {
      resolved.Diagnostics = msg.Diagnostics;
    }
    else {
      resolved.Diagnostics = ''
    }

    if (msg.PodInfo !== undefined) {
      resolved.PodInfo = msg.PodInfo;
    }
    else {
      resolved.PodInfo = 0
    }

    if (msg.HMSCheck !== undefined) {
      resolved.HMSCheck = msg.HMSCheck;
    }
    else {
      resolved.HMSCheck = 0
    }

    if (msg.OperationMode !== undefined) {
      resolved.OperationMode = msg.OperationMode;
    }
    else {
      resolved.OperationMode = 0
    }

    if (msg.PrevState !== undefined) {
      resolved.PrevState = msg.PrevState;
    }
    else {
      resolved.PrevState = 0
    }

    if (msg.CurrState !== undefined) {
      resolved.CurrState = msg.CurrState;
    }
    else {
      resolved.CurrState = 0
    }

    return resolved;
    }
};

// Constants for message
StateOut.Constants = {
  STATE_IDLE: 0,
  STATE_P2P: 1,
  STATE_IDENTIFY: 2,
  STATE_APPROACH: 3,
  STATE_VERIFY: 4,
  STATE_RETRACE: 5,
  STATE_LOCK: 6,
  STATE_UNLOCK: 7,
  STATE_EHS: 8,
  OPERATIONMODE_PICKUP: 1,
  OPERATIONMODE_DROPOFF: 2,
}

module.exports = StateOut;
