// Auto-generated. Do not edit!

// (in-package real_robot_control.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class pose {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.matrices = null;
    }
    else {
      if (initObj.hasOwnProperty('matrices')) {
        this.matrices = initObj.matrices
      }
      else {
        this.matrices = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type pose
    // Serialize message field [matrices]
    // Serialize the length for message field [matrices]
    bufferOffset = _serializer.uint32(obj.matrices.length, buffer, bufferOffset);
    obj.matrices.forEach((val) => {
      bufferOffset = std_msgs.msg.Float32MultiArray.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type pose
    let len;
    let data = new pose(null);
    // Deserialize message field [matrices]
    // Deserialize array length for message field [matrices]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.matrices = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.matrices[i] = std_msgs.msg.Float32MultiArray.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.matrices.forEach((val) => {
      length += std_msgs.msg.Float32MultiArray.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'real_robot_control/pose';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '04c3645e7964b08a28c0477b52d83e5b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Float32MultiArray[] matrices
    ================================================================================
    MSG: std_msgs/Float32MultiArray
    # Please look at the MultiArrayLayout message definition for
    # documentation on all multiarrays.
    
    MultiArrayLayout  layout        # specification of data layout
    float32[]         data          # array of data
    
    
    ================================================================================
    MSG: std_msgs/MultiArrayLayout
    # The multiarray declares a generic multi-dimensional array of a
    # particular data type.  Dimensions are ordered from outer most
    # to inner most.
    
    MultiArrayDimension[] dim # Array of dimension properties
    uint32 data_offset        # padding elements at front of data
    
    # Accessors should ALWAYS be written in terms of dimension stride
    # and specified outer-most dimension first.
    # 
    # multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
    #
    # A standard, 3-channel 640x480 image with interleaved color channels
    # would be specified as:
    #
    # dim[0].label  = "height"
    # dim[0].size   = 480
    # dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
    # dim[1].label  = "width"
    # dim[1].size   = 640
    # dim[1].stride = 3*640 = 1920
    # dim[2].label  = "channel"
    # dim[2].size   = 3
    # dim[2].stride = 3
    #
    # multiarray(i,j,k) refers to the ith row, jth column, and kth channel.
    
    ================================================================================
    MSG: std_msgs/MultiArrayDimension
    string label   # label of given dimension
    uint32 size    # size of given dimension (in type units)
    uint32 stride  # stride of given dimension
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new pose(null);
    if (msg.matrices !== undefined) {
      resolved.matrices = new Array(msg.matrices.length);
      for (let i = 0; i < resolved.matrices.length; ++i) {
        resolved.matrices[i] = std_msgs.msg.Float32MultiArray.Resolve(msg.matrices[i]);
      }
    }
    else {
      resolved.matrices = []
    }

    return resolved;
    }
};

module.exports = pose;
