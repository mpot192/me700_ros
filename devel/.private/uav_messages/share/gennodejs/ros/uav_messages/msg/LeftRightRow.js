// Auto-generated. Do not edit!

// (in-package uav_messages.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class LeftRightRow {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.left_r = null;
      this.left_theta = null;
      this.right_r = null;
      this.right_theta = null;
    }
    else {
      if (initObj.hasOwnProperty('left_r')) {
        this.left_r = initObj.left_r
      }
      else {
        this.left_r = 0.0;
      }
      if (initObj.hasOwnProperty('left_theta')) {
        this.left_theta = initObj.left_theta
      }
      else {
        this.left_theta = 0.0;
      }
      if (initObj.hasOwnProperty('right_r')) {
        this.right_r = initObj.right_r
      }
      else {
        this.right_r = 0.0;
      }
      if (initObj.hasOwnProperty('right_theta')) {
        this.right_theta = initObj.right_theta
      }
      else {
        this.right_theta = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LeftRightRow
    // Serialize message field [left_r]
    bufferOffset = _serializer.float64(obj.left_r, buffer, bufferOffset);
    // Serialize message field [left_theta]
    bufferOffset = _serializer.float64(obj.left_theta, buffer, bufferOffset);
    // Serialize message field [right_r]
    bufferOffset = _serializer.float64(obj.right_r, buffer, bufferOffset);
    // Serialize message field [right_theta]
    bufferOffset = _serializer.float64(obj.right_theta, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LeftRightRow
    let len;
    let data = new LeftRightRow(null);
    // Deserialize message field [left_r]
    data.left_r = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [left_theta]
    data.left_theta = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [right_r]
    data.right_r = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [right_theta]
    data.right_theta = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'uav_messages/LeftRightRow';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9547fcc8a1d9006ddee6af9894e1d3ac';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Most likely left/right lines in radius/angle formation.
    float64 left_r
    float64 left_theta
    float64 right_r
    float64 right_theta
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new LeftRightRow(null);
    if (msg.left_r !== undefined) {
      resolved.left_r = msg.left_r;
    }
    else {
      resolved.left_r = 0.0
    }

    if (msg.left_theta !== undefined) {
      resolved.left_theta = msg.left_theta;
    }
    else {
      resolved.left_theta = 0.0
    }

    if (msg.right_r !== undefined) {
      resolved.right_r = msg.right_r;
    }
    else {
      resolved.right_r = 0.0
    }

    if (msg.right_theta !== undefined) {
      resolved.right_theta = msg.right_theta;
    }
    else {
      resolved.right_theta = 0.0
    }

    return resolved;
    }
};

module.exports = LeftRightRow;
