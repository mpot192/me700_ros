// Auto-generated. Do not edit!

// (in-package uav_messages.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class GroundClothPoints {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.origin = null;
      this.width = null;
      this.length = null;
      this.resolution = null;
      this.heights = null;
    }
    else {
      if (initObj.hasOwnProperty('origin')) {
        this.origin = initObj.origin
      }
      else {
        this.origin = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('width')) {
        this.width = initObj.width
      }
      else {
        this.width = 0;
      }
      if (initObj.hasOwnProperty('length')) {
        this.length = initObj.length
      }
      else {
        this.length = 0;
      }
      if (initObj.hasOwnProperty('resolution')) {
        this.resolution = initObj.resolution
      }
      else {
        this.resolution = 0.0;
      }
      if (initObj.hasOwnProperty('heights')) {
        this.heights = initObj.heights
      }
      else {
        this.heights = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GroundClothPoints
    // Serialize message field [origin]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.origin, buffer, bufferOffset);
    // Serialize message field [width]
    bufferOffset = _serializer.uint32(obj.width, buffer, bufferOffset);
    // Serialize message field [length]
    bufferOffset = _serializer.uint32(obj.length, buffer, bufferOffset);
    // Serialize message field [resolution]
    bufferOffset = _serializer.float64(obj.resolution, buffer, bufferOffset);
    // Serialize message field [heights]
    bufferOffset = _arraySerializer.float64(obj.heights, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GroundClothPoints
    let len;
    let data = new GroundClothPoints(null);
    // Deserialize message field [origin]
    data.origin = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [width]
    data.width = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [length]
    data.length = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [resolution]
    data.resolution = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [heights]
    data.heights = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.heights.length;
    return length + 44;
  }

  static datatype() {
    // Returns string type for a message object
    return 'uav_messages/GroundClothPoints';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '38fbc55ea331f440db17b20331203f67';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Data alignment origin
    geometry_msgs/Vector3 origin
    uint32 width
    uint32 length
    float64 resolution
    float64[] heights
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
    const resolved = new GroundClothPoints(null);
    if (msg.origin !== undefined) {
      resolved.origin = geometry_msgs.msg.Vector3.Resolve(msg.origin)
    }
    else {
      resolved.origin = new geometry_msgs.msg.Vector3()
    }

    if (msg.width !== undefined) {
      resolved.width = msg.width;
    }
    else {
      resolved.width = 0
    }

    if (msg.length !== undefined) {
      resolved.length = msg.length;
    }
    else {
      resolved.length = 0
    }

    if (msg.resolution !== undefined) {
      resolved.resolution = msg.resolution;
    }
    else {
      resolved.resolution = 0.0
    }

    if (msg.heights !== undefined) {
      resolved.heights = msg.heights;
    }
    else {
      resolved.heights = []
    }

    return resolved;
    }
};

module.exports = GroundClothPoints;
