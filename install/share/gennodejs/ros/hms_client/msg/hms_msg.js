// Auto-generated. Do not edit!

// (in-package hms_client.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class hms_msg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.temp1 = null;
      this.temp2 = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('temp1')) {
        this.temp1 = initObj.temp1
      }
      else {
        this.temp1 = '';
      }
      if (initObj.hasOwnProperty('temp2')) {
        this.temp2 = initObj.temp2
      }
      else {
        this.temp2 = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type hms_msg
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [temp1]
    bufferOffset = _serializer.string(obj.temp1, buffer, bufferOffset);
    // Serialize message field [temp2]
    bufferOffset = _serializer.float32(obj.temp2, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type hms_msg
    let len;
    let data = new hms_msg(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [temp1]
    data.temp1 = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [temp2]
    data.temp2 = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.temp1.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'hms_client/hms_msg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '64608a45b1add361d21d3cecf93db1f6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    string temp1
    float32 temp2
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
    const resolved = new hms_msg(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.temp1 !== undefined) {
      resolved.temp1 = msg.temp1;
    }
    else {
      resolved.temp1 = ''
    }

    if (msg.temp2 !== undefined) {
      resolved.temp2 = msg.temp2;
    }
    else {
      resolved.temp2 = 0.0
    }

    return resolved;
    }
};

module.exports = hms_msg;
