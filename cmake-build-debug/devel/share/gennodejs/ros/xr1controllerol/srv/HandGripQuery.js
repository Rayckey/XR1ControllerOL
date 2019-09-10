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

class HandGripQueryRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ControlGroup = null;
    }
    else {
      if (initObj.hasOwnProperty('ControlGroup')) {
        this.ControlGroup = initObj.ControlGroup
      }
      else {
        this.ControlGroup = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type HandGripQueryRequest
    // Serialize message field [ControlGroup]
    bufferOffset = _serializer.int32(obj.ControlGroup, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type HandGripQueryRequest
    let len;
    let data = new HandGripQueryRequest(null);
    // Deserialize message field [ControlGroup]
    data.ControlGroup = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'xr1controllerol/HandGripQueryRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f8417280d351ef4d54b134c442a90906';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 ControlGroup
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new HandGripQueryRequest(null);
    if (msg.ControlGroup !== undefined) {
      resolved.ControlGroup = msg.ControlGroup;
    }
    else {
      resolved.ControlGroup = 0
    }

    return resolved;
    }
};

class HandGripQueryResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.inProgress = null;
      this.isGripped = null;
    }
    else {
      if (initObj.hasOwnProperty('inProgress')) {
        this.inProgress = initObj.inProgress
      }
      else {
        this.inProgress = false;
      }
      if (initObj.hasOwnProperty('isGripped')) {
        this.isGripped = initObj.isGripped
      }
      else {
        this.isGripped = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type HandGripQueryResponse
    // Serialize message field [inProgress]
    bufferOffset = _serializer.bool(obj.inProgress, buffer, bufferOffset);
    // Serialize message field [isGripped]
    bufferOffset = _serializer.bool(obj.isGripped, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type HandGripQueryResponse
    let len;
    let data = new HandGripQueryResponse(null);
    // Deserialize message field [inProgress]
    data.inProgress = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [isGripped]
    data.isGripped = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 2;
  }

  static datatype() {
    // Returns string type for a service object
    return 'xr1controllerol/HandGripQueryResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3820fe788ddf81a451e9d3e66e0ef8c7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool inProgress
    bool isGripped
    
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new HandGripQueryResponse(null);
    if (msg.inProgress !== undefined) {
      resolved.inProgress = msg.inProgress;
    }
    else {
      resolved.inProgress = false
    }

    if (msg.isGripped !== undefined) {
      resolved.isGripped = msg.isGripped;
    }
    else {
      resolved.isGripped = false
    }

    return resolved;
    }
};

module.exports = {
  Request: HandGripQueryRequest,
  Response: HandGripQueryResponse,
  md5sum() { return '6779f448455fbe1c7c9fa9fca9457608'; },
  datatype() { return 'xr1controllerol/HandGripQuery'; }
};
