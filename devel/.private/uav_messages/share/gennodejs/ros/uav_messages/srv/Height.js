// Auto-generated. Do not edit!

// (in-package uav_messages.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class HeightRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.points = null;
    }
    else {
      if (initObj.hasOwnProperty('points')) {
        this.points = initObj.points
      }
      else {
        this.points = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type HeightRequest
    // Serialize message field [points]
    // Serialize the length for message field [points]
    bufferOffset = _serializer.uint32(obj.points.length, buffer, bufferOffset);
    obj.points.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Vector3.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type HeightRequest
    let len;
    let data = new HeightRequest(null);
    // Deserialize message field [points]
    // Deserialize array length for message field [points]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.points = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.points[i] = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 24 * object.points.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uav_messages/HeightRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '002b2b2002a67e6ccbeb90a7022827a5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 OK=0
    uint8 SOME_INVALID=1
    
    geometry_msgs/Vector3[] points
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new HeightRequest(null);
    if (msg.points !== undefined) {
      resolved.points = new Array(msg.points.length);
      for (let i = 0; i < resolved.points.length; ++i) {
        resolved.points[i] = geometry_msgs.msg.Vector3.Resolve(msg.points[i]);
      }
    }
    else {
      resolved.points = []
    }

    return resolved;
    }
};

// Constants for message
HeightRequest.Constants = {
  OK: 0,
  SOME_INVALID: 1,
}

class HeightResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.status = null;
      this.valid = null;
      this.heights = null;
      this.mean = null;
    }
    else {
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = 0;
      }
      if (initObj.hasOwnProperty('valid')) {
        this.valid = initObj.valid
      }
      else {
        this.valid = [];
      }
      if (initObj.hasOwnProperty('heights')) {
        this.heights = initObj.heights
      }
      else {
        this.heights = [];
      }
      if (initObj.hasOwnProperty('mean')) {
        this.mean = initObj.mean
      }
      else {
        this.mean = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type HeightResponse
    // Serialize message field [status]
    bufferOffset = _serializer.uint8(obj.status, buffer, bufferOffset);
    // Serialize message field [valid]
    bufferOffset = _arraySerializer.bool(obj.valid, buffer, bufferOffset, null);
    // Serialize message field [heights]
    bufferOffset = _arraySerializer.float64(obj.heights, buffer, bufferOffset, null);
    // Serialize message field [mean]
    bufferOffset = _serializer.float64(obj.mean, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type HeightResponse
    let len;
    let data = new HeightResponse(null);
    // Deserialize message field [status]
    data.status = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [valid]
    data.valid = _arrayDeserializer.bool(buffer, bufferOffset, null)
    // Deserialize message field [heights]
    data.heights = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [mean]
    data.mean = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.valid.length;
    length += 8 * object.heights.length;
    return length + 17;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uav_messages/HeightResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f7e561783e674b1966652727cb3df7b9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 status
    bool[] valid
    float64[] heights
    float64 mean
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new HeightResponse(null);
    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = 0
    }

    if (msg.valid !== undefined) {
      resolved.valid = msg.valid;
    }
    else {
      resolved.valid = []
    }

    if (msg.heights !== undefined) {
      resolved.heights = msg.heights;
    }
    else {
      resolved.heights = []
    }

    if (msg.mean !== undefined) {
      resolved.mean = msg.mean;
    }
    else {
      resolved.mean = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: HeightRequest,
  Response: HeightResponse,
  md5sum() { return 'af6c894dddf3f73449d8d258ec820c83'; },
  datatype() { return 'uav_messages/Height'; }
};
