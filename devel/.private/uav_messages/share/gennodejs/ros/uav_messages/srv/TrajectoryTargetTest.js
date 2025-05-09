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

let trajectory_msgs = _finder('trajectory_msgs');

//-----------------------------------------------------------

class TrajectoryTargetTestRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.start = null;
      this.end = null;
    }
    else {
      if (initObj.hasOwnProperty('start')) {
        this.start = initObj.start
      }
      else {
        this.start = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('end')) {
        this.end = initObj.end
      }
      else {
        this.end = new geometry_msgs.msg.Point();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TrajectoryTargetTestRequest
    // Serialize message field [start]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.start, buffer, bufferOffset);
    // Serialize message field [end]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.end, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TrajectoryTargetTestRequest
    let len;
    let data = new TrajectoryTargetTestRequest(null);
    // Deserialize message field [start]
    data.start = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [end]
    data.end = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 48;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uav_messages/TrajectoryTargetTestRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a5a22bfbda3521cd8daea1983c7d988a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 OK=0
    uint8 ERR_NO_PATH_FOUND=1
    
    geometry_msgs/Point start
    geometry_msgs/Point end
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
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
    const resolved = new TrajectoryTargetTestRequest(null);
    if (msg.start !== undefined) {
      resolved.start = geometry_msgs.msg.Point.Resolve(msg.start)
    }
    else {
      resolved.start = new geometry_msgs.msg.Point()
    }

    if (msg.end !== undefined) {
      resolved.end = geometry_msgs.msg.Point.Resolve(msg.end)
    }
    else {
      resolved.end = new geometry_msgs.msg.Point()
    }

    return resolved;
    }
};

// Constants for message
TrajectoryTargetTestRequest.Constants = {
  OK: 0,
  ERR_NO_PATH_FOUND: 1,
}

class TrajectoryTargetTestResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.error = null;
      this.runtime_us = null;
      this.traj = null;
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
      if (initObj.hasOwnProperty('runtime_us')) {
        this.runtime_us = initObj.runtime_us
      }
      else {
        this.runtime_us = 0;
      }
      if (initObj.hasOwnProperty('traj')) {
        this.traj = initObj.traj
      }
      else {
        this.traj = new trajectory_msgs.msg.MultiDOFJointTrajectory();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TrajectoryTargetTestResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [error]
    bufferOffset = _serializer.uint8(obj.error, buffer, bufferOffset);
    // Serialize message field [runtime_us]
    bufferOffset = _serializer.uint32(obj.runtime_us, buffer, bufferOffset);
    // Serialize message field [traj]
    bufferOffset = trajectory_msgs.msg.MultiDOFJointTrajectory.serialize(obj.traj, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TrajectoryTargetTestResponse
    let len;
    let data = new TrajectoryTargetTestResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [error]
    data.error = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [runtime_us]
    data.runtime_us = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [traj]
    data.traj = trajectory_msgs.msg.MultiDOFJointTrajectory.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += trajectory_msgs.msg.MultiDOFJointTrajectory.getMessageSize(object.traj);
    return length + 6;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uav_messages/TrajectoryTargetTestResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '393baf917af587233c07fc1fdd6c6fe2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    uint8 error
    uint32 runtime_us
    trajectory_msgs/MultiDOFJointTrajectory traj
    
    ================================================================================
    MSG: trajectory_msgs/MultiDOFJointTrajectory
    # The header is used to specify the coordinate frame and the reference time for the trajectory durations
    Header header
    
    # A representation of a multi-dof joint trajectory (each point is a transformation)
    # Each point along the trajectory will include an array of positions/velocities/accelerations
    # that has the same length as the array of joint names, and has the same order of joints as 
    # the joint names array.
    
    string[] joint_names
    MultiDOFJointTrajectoryPoint[] points
    
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
    
    ================================================================================
    MSG: trajectory_msgs/MultiDOFJointTrajectoryPoint
    # Each multi-dof joint can specify a transform (up to 6 DOF)
    geometry_msgs/Transform[] transforms
    
    # There can be a velocity specified for the origin of the joint 
    geometry_msgs/Twist[] velocities
    
    # There can be an acceleration specified for the origin of the joint 
    geometry_msgs/Twist[] accelerations
    
    duration time_from_start
    
    ================================================================================
    MSG: geometry_msgs/Transform
    # This represents the transform between two coordinate frames in free space.
    
    Vector3 translation
    Quaternion rotation
    
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
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    ================================================================================
    MSG: geometry_msgs/Twist
    # This expresses velocity in free space broken into its linear and angular parts.
    Vector3  linear
    Vector3  angular
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TrajectoryTargetTestResponse(null);
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

    if (msg.runtime_us !== undefined) {
      resolved.runtime_us = msg.runtime_us;
    }
    else {
      resolved.runtime_us = 0
    }

    if (msg.traj !== undefined) {
      resolved.traj = trajectory_msgs.msg.MultiDOFJointTrajectory.Resolve(msg.traj)
    }
    else {
      resolved.traj = new trajectory_msgs.msg.MultiDOFJointTrajectory()
    }

    return resolved;
    }
};

module.exports = {
  Request: TrajectoryTargetTestRequest,
  Response: TrajectoryTargetTestResponse,
  md5sum() { return '6aa66adc9f9e06718260a8729c57876c'; },
  datatype() { return 'uav_messages/TrajectoryTargetTest'; }
};
