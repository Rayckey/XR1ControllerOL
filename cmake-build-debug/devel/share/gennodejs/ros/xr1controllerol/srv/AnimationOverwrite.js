// Auto-generated. Do not edit!

// (in-package xr1controllerol.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class AnimationOverwriteRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.AnimationType = null;
      this.AnimationID = null;
      this.AnimationData = null;
    }
    else {
      if (initObj.hasOwnProperty('AnimationType')) {
        this.AnimationType = initObj.AnimationType
      }
      else {
        this.AnimationType = 0;
      }
      if (initObj.hasOwnProperty('AnimationID')) {
        this.AnimationID = initObj.AnimationID
      }
      else {
        this.AnimationID = 0;
      }
      if (initObj.hasOwnProperty('AnimationData')) {
        this.AnimationData = initObj.AnimationData
      }
      else {
        this.AnimationData = new std_msgs.msg.Float64MultiArray();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AnimationOverwriteRequest
    // Serialize message field [AnimationType]
    bufferOffset = _serializer.int32(obj.AnimationType, buffer, bufferOffset);
    // Serialize message field [AnimationID]
    bufferOffset = _serializer.int64(obj.AnimationID, buffer, bufferOffset);
    // Serialize message field [AnimationData]
    bufferOffset = std_msgs.msg.Float64MultiArray.serialize(obj.AnimationData, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AnimationOverwriteRequest
    let len;
    let data = new AnimationOverwriteRequest(null);
    // Deserialize message field [AnimationType]
    data.AnimationType = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [AnimationID]
    data.AnimationID = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [AnimationData]
    data.AnimationData = std_msgs.msg.Float64MultiArray.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Float64MultiArray.getMessageSize(object.AnimationData);
    return length + 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'xr1controllerol/AnimationOverwriteRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '60b112eb5c8607f7e01731a0c0d96d0c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 AnimationType
    int64 AnimationID
    std_msgs/Float64MultiArray AnimationData
    
    ================================================================================
    MSG: std_msgs/Float64MultiArray
    # Please look at the MultiArrayLayout message definition for
    # documentation on all multiarrays.
    
    MultiArrayLayout  layout        # specification of data layout
    float64[]         data          # array of data
    
    
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
    const resolved = new AnimationOverwriteRequest(null);
    if (msg.AnimationType !== undefined) {
      resolved.AnimationType = msg.AnimationType;
    }
    else {
      resolved.AnimationType = 0
    }

    if (msg.AnimationID !== undefined) {
      resolved.AnimationID = msg.AnimationID;
    }
    else {
      resolved.AnimationID = 0
    }

    if (msg.AnimationData !== undefined) {
      resolved.AnimationData = std_msgs.msg.Float64MultiArray.Resolve(msg.AnimationData)
    }
    else {
      resolved.AnimationData = new std_msgs.msg.Float64MultiArray()
    }

    return resolved;
    }
};

class AnimationOverwriteResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.isLoaded = null;
    }
    else {
      if (initObj.hasOwnProperty('isLoaded')) {
        this.isLoaded = initObj.isLoaded
      }
      else {
        this.isLoaded = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AnimationOverwriteResponse
    // Serialize message field [isLoaded]
    bufferOffset = _serializer.bool(obj.isLoaded, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AnimationOverwriteResponse
    let len;
    let data = new AnimationOverwriteResponse(null);
    // Deserialize message field [isLoaded]
    data.isLoaded = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'xr1controllerol/AnimationOverwriteResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '734cdc90acd0ec13d38631e052987af8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool isLoaded
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AnimationOverwriteResponse(null);
    if (msg.isLoaded !== undefined) {
      resolved.isLoaded = msg.isLoaded;
    }
    else {
      resolved.isLoaded = false
    }

    return resolved;
    }
};

module.exports = {
  Request: AnimationOverwriteRequest,
  Response: AnimationOverwriteResponse,
  md5sum() { return '7243db66331b06264c2b466bb4f193aa'; },
  datatype() { return 'xr1controllerol/AnimationOverwrite'; }
};
