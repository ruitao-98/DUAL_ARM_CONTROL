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

class gripper {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.open = null;
    }
    else {
      if (initObj.hasOwnProperty('open')) {
        this.open = initObj.open
      }
      else {
        this.open = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type gripper
    // Serialize message field [open]
    bufferOffset = _serializer.float64(obj.open, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type gripper
    let len;
    let data = new gripper(null);
    // Deserialize message field [open]
    data.open = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'real_robot_control/gripper';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1cd3c8c0899095b58f167c1f2be4d95a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 open
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new gripper(null);
    if (msg.open !== undefined) {
      resolved.open = msg.open;
    }
    else {
      resolved.open = 0.0
    }

    return resolved;
    }
};

module.exports = gripper;
