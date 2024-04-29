// Auto-generated. Do not edit!

// (in-package my_listener.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class TimeTransform {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.geo_trans = null;
      this.t1 = null;
      this.t2 = null;
    }
    else {
      if (initObj.hasOwnProperty('geo_trans')) {
        this.geo_trans = initObj.geo_trans
      }
      else {
        this.geo_trans = new geometry_msgs.msg.Transform();
      }
      if (initObj.hasOwnProperty('t1')) {
        this.t1 = initObj.t1
      }
      else {
        this.t1 = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('t2')) {
        this.t2 = initObj.t2
      }
      else {
        this.t2 = {secs: 0, nsecs: 0};
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TimeTransform
    // Serialize message field [geo_trans]
    bufferOffset = geometry_msgs.msg.Transform.serialize(obj.geo_trans, buffer, bufferOffset);
    // Serialize message field [t1]
    bufferOffset = _serializer.time(obj.t1, buffer, bufferOffset);
    // Serialize message field [t2]
    bufferOffset = _serializer.time(obj.t2, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TimeTransform
    let len;
    let data = new TimeTransform(null);
    // Deserialize message field [geo_trans]
    data.geo_trans = geometry_msgs.msg.Transform.deserialize(buffer, bufferOffset);
    // Deserialize message field [t1]
    data.t1 = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [t2]
    data.t2 = _deserializer.time(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 72;
  }

  static datatype() {
    // Returns string type for a message object
    return 'my_listener/TimeTransform';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b05693e338e8c00beb115d4348be07e1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Transform geo_trans
    time t1
    time t2
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TimeTransform(null);
    if (msg.geo_trans !== undefined) {
      resolved.geo_trans = geometry_msgs.msg.Transform.Resolve(msg.geo_trans)
    }
    else {
      resolved.geo_trans = new geometry_msgs.msg.Transform()
    }

    if (msg.t1 !== undefined) {
      resolved.t1 = msg.t1;
    }
    else {
      resolved.t1 = {secs: 0, nsecs: 0}
    }

    if (msg.t2 !== undefined) {
      resolved.t2 = msg.t2;
    }
    else {
      resolved.t2 = {secs: 0, nsecs: 0}
    }

    return resolved;
    }
};

module.exports = TimeTransform;
