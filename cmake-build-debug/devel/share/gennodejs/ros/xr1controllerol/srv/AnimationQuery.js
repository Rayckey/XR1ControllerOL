// Auto-generated. Do not edit!

// (in-package xr1controllerol.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class AnimationQueryRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.isQuery = null;
    }
    else {
      if (initObj.hasOwnProperty('isQuery')) {
        this.isQuery = initObj.isQuery
      }
      else {
        this.isQuery = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AnimationQueryRequest
    // Serialize message field [isQuery]
    bufferOffset = _serializer.bool(obj.isQuery, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AnimationQueryRequest
    let len;
    let data = new AnimationQueryRequest(null);
    // Deserialize message field [isQuery]
    data.isQuery = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'xr1controllerol/AnimationQueryRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5f343d8e152482379fa3d057150c64e6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool isQuery
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AnimationQueryRequest(null);
    if (msg.isQuery !== undefined) {
      resolved.isQuery = msg.isQuery;
    }
    else {
      resolved.isQuery = false
    }

    return resolved;
    }
};

class AnimationQueryResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.inAnimationMode = null;
      this.isPlaying = null;
      this.hasDefault = null;
      this.hasIdle = null;
      this.AnimationType = null;
      this.AnimationID = null;
      this.AnimationProgress = null;
    }
    else {
      if (initObj.hasOwnProperty('inAnimationMode')) {
        this.inAnimationMode = initObj.inAnimationMode
      }
      else {
        this.inAnimationMode = false;
      }
      if (initObj.hasOwnProperty('isPlaying')) {
        this.isPlaying = initObj.isPlaying
      }
      else {
        this.isPlaying = false;
      }
      if (initObj.hasOwnProperty('hasDefault')) {
        this.hasDefault = initObj.hasDefault
      }
      else {
        this.hasDefault = false;
      }
      if (initObj.hasOwnProperty('hasIdle')) {
        this.hasIdle = initObj.hasIdle
      }
      else {
        this.hasIdle = false;
      }
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
      if (initObj.hasOwnProperty('AnimationProgress')) {
        this.AnimationProgress = initObj.AnimationProgress
      }
      else {
        this.AnimationProgress = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AnimationQueryResponse
    // Serialize message field [inAnimationMode]
    bufferOffset = _serializer.bool(obj.inAnimationMode, buffer, bufferOffset);
    // Serialize message field [isPlaying]
    bufferOffset = _serializer.bool(obj.isPlaying, buffer, bufferOffset);
    // Serialize message field [hasDefault]
    bufferOffset = _serializer.bool(obj.hasDefault, buffer, bufferOffset);
    // Serialize message field [hasIdle]
    bufferOffset = _serializer.bool(obj.hasIdle, buffer, bufferOffset);
    // Serialize message field [AnimationType]
    bufferOffset = _serializer.int64(obj.AnimationType, buffer, bufferOffset);
    // Serialize message field [AnimationID]
    bufferOffset = _serializer.int64(obj.AnimationID, buffer, bufferOffset);
    // Serialize message field [AnimationProgress]
    bufferOffset = _serializer.int64(obj.AnimationProgress, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AnimationQueryResponse
    let len;
    let data = new AnimationQueryResponse(null);
    // Deserialize message field [inAnimationMode]
    data.inAnimationMode = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [isPlaying]
    data.isPlaying = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [hasDefault]
    data.hasDefault = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [hasIdle]
    data.hasIdle = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [AnimationType]
    data.AnimationType = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [AnimationID]
    data.AnimationID = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [AnimationProgress]
    data.AnimationProgress = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 28;
  }

  static datatype() {
    // Returns string type for a service object
    return 'xr1controllerol/AnimationQueryResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2103b70505eb43626eb30821dba4afa4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool inAnimationMode
    bool isPlaying
    bool hasDefault
    bool hasIdle
    int64 AnimationType
    int64 AnimationID
    int64 AnimationProgress
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AnimationQueryResponse(null);
    if (msg.inAnimationMode !== undefined) {
      resolved.inAnimationMode = msg.inAnimationMode;
    }
    else {
      resolved.inAnimationMode = false
    }

    if (msg.isPlaying !== undefined) {
      resolved.isPlaying = msg.isPlaying;
    }
    else {
      resolved.isPlaying = false
    }

    if (msg.hasDefault !== undefined) {
      resolved.hasDefault = msg.hasDefault;
    }
    else {
      resolved.hasDefault = false
    }

    if (msg.hasIdle !== undefined) {
      resolved.hasIdle = msg.hasIdle;
    }
    else {
      resolved.hasIdle = false
    }

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

    if (msg.AnimationProgress !== undefined) {
      resolved.AnimationProgress = msg.AnimationProgress;
    }
    else {
      resolved.AnimationProgress = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: AnimationQueryRequest,
  Response: AnimationQueryResponse,
  md5sum() { return '5b670fef922a4d6388f8dea2bff6ef00'; },
  datatype() { return 'xr1controllerol/AnimationQuery'; }
};
