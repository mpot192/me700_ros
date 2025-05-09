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

class TrajectoryFollowerStatusRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.request = null;
    }
    else {
      if (initObj.hasOwnProperty('request')) {
        this.request = initObj.request
      }
      else {
        this.request = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TrajectoryFollowerStatusRequest
    // Serialize message field [request]
    bufferOffset = _serializer.bool(obj.request, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TrajectoryFollowerStatusRequest
    let len;
    let data = new TrajectoryFollowerStatusRequest(null);
    // Deserialize message field [request]
    data.request = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uav_messages/TrajectoryFollowerStatusRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a43de1986dc2edeec1fcf33497198a04';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 IDLE=0
    uint8 FOLLOWING=1
    
    bool request
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TrajectoryFollowerStatusRequest(null);
    if (msg.request !== undefined) {
      resolved.request = msg.request;
    }
    else {
      resolved.request = false
    }

    return resolved;
    }
};

// Constants for message
TrajectoryFollowerStatusRequest.Constants = {
  IDLE: 0,
  FOLLOWING: 1,
}

class TrajectoryFollowerStatusResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.status = null;
    }
    else {
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TrajectoryFollowerStatusResponse
    // Serialize message field [status]
    bufferOffset = _serializer.uint8(obj.status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TrajectoryFollowerStatusResponse
    let len;
    let data = new TrajectoryFollowerStatusResponse(null);
    // Deserialize message field [status]
    data.status = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uav_messages/TrajectoryFollowerStatusResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '284aa12dd9e9e760802ac9f38036ea5e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 status
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TrajectoryFollowerStatusResponse(null);
    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: TrajectoryFollowerStatusRequest,
  Response: TrajectoryFollowerStatusResponse,
  md5sum() { return '29cf0115501ebd4739c6b692f76b7c0e'; },
  datatype() { return 'uav_messages/TrajectoryFollowerStatus'; }
};
