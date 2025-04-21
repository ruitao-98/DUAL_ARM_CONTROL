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

class ori_adj_rec {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.phi = null;
      this.point_num = null;
      this.record_item = null;
      this.Rx = null;
      this.Ry = null;
      this.Rz = null;
    }
    else {
      if (initObj.hasOwnProperty('phi')) {
        this.phi = initObj.phi
      }
      else {
        this.phi = 0;
      }
      if (initObj.hasOwnProperty('point_num')) {
        this.point_num = initObj.point_num
      }
      else {
        this.point_num = 0;
      }
      if (initObj.hasOwnProperty('record_item')) {
        this.record_item = initObj.record_item
      }
      else {
        this.record_item = 0;
      }
      if (initObj.hasOwnProperty('Rx')) {
        this.Rx = initObj.Rx
      }
      else {
        this.Rx = 0.0;
      }
      if (initObj.hasOwnProperty('Ry')) {
        this.Ry = initObj.Ry
      }
      else {
        this.Ry = 0.0;
      }
      if (initObj.hasOwnProperty('Rz')) {
        this.Rz = initObj.Rz
      }
      else {
        this.Rz = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ori_adj_rec
    // Serialize message field [phi]
    bufferOffset = _serializer.int32(obj.phi, buffer, bufferOffset);
    // Serialize message field [point_num]
    bufferOffset = _serializer.int32(obj.point_num, buffer, bufferOffset);
    // Serialize message field [record_item]
    bufferOffset = _serializer.int32(obj.record_item, buffer, bufferOffset);
    // Serialize message field [Rx]
    bufferOffset = _serializer.float64(obj.Rx, buffer, bufferOffset);
    // Serialize message field [Ry]
    bufferOffset = _serializer.float64(obj.Ry, buffer, bufferOffset);
    // Serialize message field [Rz]
    bufferOffset = _serializer.float64(obj.Rz, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ori_adj_rec
    let len;
    let data = new ori_adj_rec(null);
    // Deserialize message field [phi]
    data.phi = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [point_num]
    data.point_num = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [record_item]
    data.record_item = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [Rx]
    data.Rx = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Ry]
    data.Ry = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Rz]
    data.Rz = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 36;
  }

  static datatype() {
    // Returns string type for a message object
    return 'real_robot_control/ori_adj_rec';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '41fd7db046eaccfb5112edd2ddb827ba';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 phi
    int32 point_num
    int32 record_item
    float64 Rx
    float64 Ry
    float64 Rz
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ori_adj_rec(null);
    if (msg.phi !== undefined) {
      resolved.phi = msg.phi;
    }
    else {
      resolved.phi = 0
    }

    if (msg.point_num !== undefined) {
      resolved.point_num = msg.point_num;
    }
    else {
      resolved.point_num = 0
    }

    if (msg.record_item !== undefined) {
      resolved.record_item = msg.record_item;
    }
    else {
      resolved.record_item = 0
    }

    if (msg.Rx !== undefined) {
      resolved.Rx = msg.Rx;
    }
    else {
      resolved.Rx = 0.0
    }

    if (msg.Ry !== undefined) {
      resolved.Ry = msg.Ry;
    }
    else {
      resolved.Ry = 0.0
    }

    if (msg.Rz !== undefined) {
      resolved.Rz = msg.Rz;
    }
    else {
      resolved.Rz = 0.0
    }

    return resolved;
    }
};

module.exports = ori_adj_rec;
