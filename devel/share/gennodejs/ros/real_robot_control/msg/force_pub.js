// Auto-generated. Do not edit!

// (in-package real_robot_control.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class force_pub {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.X = null;
      this.Y = null;
      this.Z = null;
      this.MX = null;
      this.MY = null;
      this.MZ = null;
    }
    else {
      if (initObj.hasOwnProperty('X')) {
        this.X = initObj.X
      }
      else {
        this.X = 0.0;
      }
      if (initObj.hasOwnProperty('Y')) {
        this.Y = initObj.Y
      }
      else {
        this.Y = 0.0;
      }
      if (initObj.hasOwnProperty('Z')) {
        this.Z = initObj.Z
      }
      else {
        this.Z = 0.0;
      }
      if (initObj.hasOwnProperty('MX')) {
        this.MX = initObj.MX
      }
      else {
        this.MX = 0.0;
      }
      if (initObj.hasOwnProperty('MY')) {
        this.MY = initObj.MY
      }
      else {
        this.MY = 0.0;
      }
      if (initObj.hasOwnProperty('MZ')) {
        this.MZ = initObj.MZ
      }
      else {
        this.MZ = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type force_pub
    // Serialize message field [X]
    bufferOffset = _serializer.float64(obj.X, buffer, bufferOffset);
    // Serialize message field [Y]
    bufferOffset = _serializer.float64(obj.Y, buffer, bufferOffset);
    // Serialize message field [Z]
    bufferOffset = _serializer.float64(obj.Z, buffer, bufferOffset);
    // Serialize message field [MX]
    bufferOffset = _serializer.float64(obj.MX, buffer, bufferOffset);
    // Serialize message field [MY]
    bufferOffset = _serializer.float64(obj.MY, buffer, bufferOffset);
    // Serialize message field [MZ]
    bufferOffset = _serializer.float64(obj.MZ, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type force_pub
    let len;
    let data = new force_pub(null);
    // Deserialize message field [X]
    data.X = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Y]
    data.Y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Z]
    data.Z = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [MX]
    data.MX = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [MY]
    data.MY = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [MZ]
    data.MZ = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 48;
  }

  static datatype() {
    // Returns string type for a message object
    return 'real_robot_control/force_pub';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '01bd0bd3e5758946edad85592cef21e2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 X
    float64 Y
    float64 Z
    float64 MX
    float64 MY
    float64 MZ
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new force_pub(null);
    if (msg.X !== undefined) {
      resolved.X = msg.X;
    }
    else {
      resolved.X = 0.0
    }

    if (msg.Y !== undefined) {
      resolved.Y = msg.Y;
    }
    else {
      resolved.Y = 0.0
    }

    if (msg.Z !== undefined) {
      resolved.Z = msg.Z;
    }
    else {
      resolved.Z = 0.0
    }

    if (msg.MX !== undefined) {
      resolved.MX = msg.MX;
    }
    else {
      resolved.MX = 0.0
    }

    if (msg.MY !== undefined) {
      resolved.MY = msg.MY;
    }
    else {
      resolved.MY = 0.0
    }

    if (msg.MZ !== undefined) {
      resolved.MZ = msg.MZ;
    }
    else {
      resolved.MZ = 0.0
    }

    return resolved;
    }
};

module.exports = force_pub;
