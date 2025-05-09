// Auto-generated. Do not edit!

// (in-package uav_messages.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class HeartbeatRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.heartbeat = null;
    }
    else {
      if (initObj.hasOwnProperty('heartbeat')) {
        this.heartbeat = initObj.heartbeat
      }
      else {
        this.heartbeat = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type HeartbeatRequest
    // Serialize message field [heartbeat]
    bufferOffset = _serializer.bool(obj.heartbeat, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type HeartbeatRequest
    let len;
    let data = new HeartbeatRequest(null);
    // Deserialize message field [heartbeat]
    data.heartbeat = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uav_messages/HeartbeatRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '200e4430827ea87384fb21872bb9ff37';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool heartbeat
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new HeartbeatRequest(null);
    if (msg.heartbeat !== undefined) {
      resolved.heartbeat = msg.heartbeat;
    }
    else {
      resolved.heartbeat = false
    }

    return resolved;
    }
};

class HeartbeatResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.heartbeat = null;
    }
    else {
      if (initObj.hasOwnProperty('heartbeat')) {
        this.heartbeat = initObj.heartbeat
      }
      else {
        this.heartbeat = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type HeartbeatResponse
    // Serialize message field [heartbeat]
    bufferOffset = _serializer.bool(obj.heartbeat, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type HeartbeatResponse
    let len;
    let data = new HeartbeatResponse(null);
    // Deserialize message field [heartbeat]
    data.heartbeat = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uav_messages/HeartbeatResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '200e4430827ea87384fb21872bb9ff37';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool heartbeat
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new HeartbeatResponse(null);
    if (msg.heartbeat !== undefined) {
      resolved.heartbeat = msg.heartbeat;
    }
    else {
      resolved.heartbeat = false
    }

    return resolved;
    }
};

module.exports = {
  Request: HeartbeatRequest,
  Response: HeartbeatResponse,
  md5sum() { return '927048d99b53abc440d02d20ffdd561a'; },
  datatype() { return 'uav_messages/Heartbeat'; }
};
