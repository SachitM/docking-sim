// Auto-generated. Do not edit!

// (in-package hms_client.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let hms_msg = require('../msg/hms_msg.js');

//-----------------------------------------------------------

class ping_pongRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.node_name = null;
    }
    else {
      if (initObj.hasOwnProperty('node_name')) {
        this.node_name = initObj.node_name
      }
      else {
        this.node_name = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ping_pongRequest
    // Serialize message field [node_name]
    bufferOffset = _serializer.string(obj.node_name, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ping_pongRequest
    let len;
    let data = new ping_pongRequest(null);
    // Deserialize message field [node_name]
    data.node_name = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.node_name.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'hms_client/ping_pongRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '46dbb9fd4d6116d8efbaf5fa2a959582';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #request 
    string node_name
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ping_pongRequest(null);
    if (msg.node_name !== undefined) {
      resolved.node_name = msg.node_name;
    }
    else {
      resolved.node_name = ''
    }

    return resolved;
    }
};

class ping_pongResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.msg = null;
      this.health = null;
      this.error_code = null;
    }
    else {
      if (initObj.hasOwnProperty('msg')) {
        this.msg = initObj.msg
      }
      else {
        this.msg = new hms_msg();
      }
      if (initObj.hasOwnProperty('health')) {
        this.health = initObj.health
      }
      else {
        this.health = 0;
      }
      if (initObj.hasOwnProperty('error_code')) {
        this.error_code = initObj.error_code
      }
      else {
        this.error_code = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ping_pongResponse
    // Serialize message field [msg]
    bufferOffset = hms_msg.serialize(obj.msg, buffer, bufferOffset);
    // Serialize message field [health]
    bufferOffset = _serializer.int64(obj.health, buffer, bufferOffset);
    // Serialize message field [error_code]
    bufferOffset = _serializer.int64(obj.error_code, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ping_pongResponse
    let len;
    let data = new ping_pongResponse(null);
    // Deserialize message field [msg]
    data.msg = hms_msg.deserialize(buffer, bufferOffset);
    // Deserialize message field [health]
    data.health = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [error_code]
    data.error_code = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += hms_msg.getMessageSize(object.msg);
    return length + 16;
  }

  static datatype() {
    // Returns string type for a service object
    return 'hms_client/ping_pongResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd7b86f9eb0923993e40e04c44b2dc528';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #response 
    hms_msg msg
    int64 health
    int64 error_code
    
    
    
    ================================================================================
    MSG: hms_client/hms_msg
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
    const resolved = new ping_pongResponse(null);
    if (msg.msg !== undefined) {
      resolved.msg = hms_msg.Resolve(msg.msg)
    }
    else {
      resolved.msg = new hms_msg()
    }

    if (msg.health !== undefined) {
      resolved.health = msg.health;
    }
    else {
      resolved.health = 0
    }

    if (msg.error_code !== undefined) {
      resolved.error_code = msg.error_code;
    }
    else {
      resolved.error_code = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: ping_pongRequest,
  Response: ping_pongResponse,
  md5sum() { return 'f8a6e80cda99ecae7bfeb48f745404ba'; },
  datatype() { return 'hms_client/ping_pong'; }
};
