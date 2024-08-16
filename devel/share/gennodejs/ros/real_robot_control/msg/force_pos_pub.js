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

class force_pos_pub {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.FX = null;
      this.FY = null;
      this.FZ = null;
      this.X = null;
      this.Y = null;
      this.Z = null;
      this.theta = null;
    }
    else {
      if (initObj.hasOwnProperty('FX')) {
        this.FX = initObj.FX
      }
      else {
        this.FX = 0.0;
      }
      if (initObj.hasOwnProperty('FY')) {
        this.FY = initObj.FY
      }
      else {
        this.FY = 0.0;
      }
      if (initObj.hasOwnProperty('FZ')) {
        this.FZ = initObj.FZ
      }
      else {
        this.FZ = 0.0;
      }
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
      if (initObj.hasOwnProperty('theta')) {
        this.theta = initObj.theta
      }
      else {
        this.theta = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type force_pos_pub
    // Serialize message field [FX]
    bufferOffset = _serializer.float64(obj.FX, buffer, bufferOffset);
    // Serialize message field [FY]
    bufferOffset = _serializer.float64(obj.FY, buffer, bufferOffset);
    // Serialize message field [FZ]
    bufferOffset = _serializer.float64(obj.FZ, buffer, bufferOffset);
    // Serialize message field [X]
    bufferOffset = _serializer.float64(obj.X, buffer, bufferOffset);
    // Serialize message field [Y]
    bufferOffset = _serializer.float64(obj.Y, buffer, bufferOffset);
    // Serialize message field [Z]
    bufferOffset = _serializer.float64(obj.Z, buffer, bufferOffset);
    // Serialize message field [theta]
    bufferOffset = _serializer.float64(obj.theta, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type force_pos_pub
    let len;
    let data = new force_pos_pub(null);
    // Deserialize message field [FX]
    data.FX = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [FY]
    data.FY = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [FZ]
    data.FZ = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [X]
    data.X = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Y]
    data.Y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Z]
    data.Z = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [theta]
    data.theta = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 56;
  }

  static datatype() {
    // Returns string type for a message object
    return 'real_robot_control/force_pos_pub';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '602988f5260503914aa61c83fb41c15c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 FX
    float64 FY
    float64 FZ
    float64 X
    float64 Y
    float64 Z
    float64 theta
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new force_pos_pub(null);
    if (msg.FX !== undefined) {
      resolved.FX = msg.FX;
    }
    else {
      resolved.FX = 0.0
    }

    if (msg.FY !== undefined) {
      resolved.FY = msg.FY;
    }
    else {
      resolved.FY = 0.0
    }

    if (msg.FZ !== undefined) {
      resolved.FZ = msg.FZ;
    }
    else {
      resolved.FZ = 0.0
    }

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

    if (msg.theta !== undefined) {
      resolved.theta = msg.theta;
    }
    else {
      resolved.theta = 0.0
    }

    return resolved;
    }
};

module.exports = force_pos_pub;
