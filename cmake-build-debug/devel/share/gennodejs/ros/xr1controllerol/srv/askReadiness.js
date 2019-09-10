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

class askReadinessRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.isAsking = null;
    }
    else {
      if (initObj.hasOwnProperty('isAsking')) {
        this.isAsking = initObj.isAsking
      }
      else {
        this.isAsking = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type askReadinessRequest
    // Serialize message field [isAsking]
    bufferOffset = _serializer.bool(obj.isAsking, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type askReadinessRequest
    let len;
    let data = new askReadinessRequest(null);
    // Deserialize message field [isAsking]
    data.isAsking = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'xr1controllerol/askReadinessRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '51203b59eff78db56086bfc4b961052a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool isAsking
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new askReadinessRequest(null);
    if (msg.isAsking !== undefined) {
      resolved.isAsking = msg.isAsking;
    }
    else {
      resolved.isAsking = false
    }

    return resolved;
    }
};

class askReadinessResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.isReady = null;
    }
    else {
      if (initObj.hasOwnProperty('isReady')) {
        this.isReady = initObj.isReady
      }
      else {
        this.isReady = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type askReadinessResponse
    // Serialize message field [isReady]
    bufferOffset = _serializer.bool(obj.isReady, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type askReadinessResponse
    let len;
    let data = new askReadinessResponse(null);
    // Deserialize message field [isReady]
    data.isReady = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'xr1controllerol/askReadinessResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd49eca18419caa818b5f22250810ca92';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool isReady
    
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new askReadinessResponse(null);
    if (msg.isReady !== undefined) {
      resolved.isReady = msg.isReady;
    }
    else {
      resolved.isReady = false
    }

    return resolved;
    }
};

module.exports = {
  Request: askReadinessRequest,
  Response: askReadinessResponse,
  md5sum() { return '4a4751acb64765569565cc0a23cfd7be'; },
  datatype() { return 'xr1controllerol/askReadiness'; }
};
