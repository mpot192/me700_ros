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

class TrajectoryTargetRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.waypoints = null;
      this.start_velocity = null;
      this.end_velocity = null;
      this.target_velocity = null;
    }
    else {
      if (initObj.hasOwnProperty('waypoints')) {
        this.waypoints = initObj.waypoints
      }
      else {
        this.waypoints = [];
      }
      if (initObj.hasOwnProperty('start_velocity')) {
        this.start_velocity = initObj.start_velocity
      }
      else {
        this.start_velocity = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('end_velocity')) {
        this.end_velocity = initObj.end_velocity
      }
      else {
        this.end_velocity = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('target_velocity')) {
        this.target_velocity = initObj.target_velocity
      }
      else {
        this.target_velocity = new geometry_msgs.msg.Vector3();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TrajectoryTargetRequest
    // Serialize message field [waypoints]
    // Serialize the length for message field [waypoints]
    bufferOffset = _serializer.uint32(obj.waypoints.length, buffer, bufferOffset);
    obj.waypoints.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [start_velocity]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.start_velocity, buffer, bufferOffset);
    // Serialize message field [end_velocity]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.end_velocity, buffer, bufferOffset);
    // Serialize message field [target_velocity]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.target_velocity, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TrajectoryTargetRequest
    let len;
    let data = new TrajectoryTargetRequest(null);
    // Deserialize message field [waypoints]
    // Deserialize array length for message field [waypoints]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.waypoints = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.waypoints[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [start_velocity]
    data.start_velocity = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [end_velocity]
    data.end_velocity = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [target_velocity]
    data.target_velocity = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 24 * object.waypoints.length;
    return length + 76;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uav_messages/TrajectoryTargetRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '512cc6ddbabf522dc3869f10476712b3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 OK=0
    uint8 ERR_NO_PATH_FOUND=1
    
    geometry_msgs/Point[] waypoints
    geometry_msgs/Vector3 start_velocity
    geometry_msgs/Vector3 end_velocity
    geometry_msgs/Vector3 target_velocity
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
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
    const resolved = new TrajectoryTargetRequest(null);
    if (msg.waypoints !== undefined) {
      resolved.waypoints = new Array(msg.waypoints.length);
      for (let i = 0; i < resolved.waypoints.length; ++i) {
        resolved.waypoints[i] = geometry_msgs.msg.Point.Resolve(msg.waypoints[i]);
      }
    }
    else {
      resolved.waypoints = []
    }

    if (msg.start_velocity !== undefined) {
      resolved.start_velocity = geometry_msgs.msg.Vector3.Resolve(msg.start_velocity)
    }
    else {
      resolved.start_velocity = new geometry_msgs.msg.Vector3()
    }

    if (msg.end_velocity !== undefined) {
      resolved.end_velocity = geometry_msgs.msg.Vector3.Resolve(msg.end_velocity)
    }
    else {
      resolved.end_velocity = new geometry_msgs.msg.Vector3()
    }

    if (msg.target_velocity !== undefined) {
      resolved.target_velocity = geometry_msgs.msg.Vector3.Resolve(msg.target_velocity)
    }
    else {
      resolved.target_velocity = new geometry_msgs.msg.Vector3()
    }

    return resolved;
    }
};

// Constants for message
TrajectoryTargetRequest.Constants = {
  OK: 0,
  ERR_NO_PATH_FOUND: 1,
}

class TrajectoryTargetResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.error = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('error')) {
        this.error = initObj.error
      }
      else {
        this.error = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TrajectoryTargetResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [error]
    bufferOffset = _serializer.uint8(obj.error, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TrajectoryTargetResponse
    let len;
    let data = new TrajectoryTargetResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [error]
    data.error = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 2;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uav_messages/TrajectoryTargetResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '21f580d3708977aeeb985babbeadc7e5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    uint8 error
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TrajectoryTargetResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.error !== undefined) {
      resolved.error = msg.error;
    }
    else {
      resolved.error = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: TrajectoryTargetRequest,
  Response: TrajectoryTargetResponse,
  md5sum() { return 'ccd1ece0eed8a932ade7a0f997a2a505'; },
  datatype() { return 'uav_messages/TrajectoryTarget'; }
};
