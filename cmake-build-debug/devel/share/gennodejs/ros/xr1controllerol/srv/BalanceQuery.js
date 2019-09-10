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

class BalanceQueryRequest {
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
    // Serializes a message object of type BalanceQueryRequest
    // Serialize message field [isQuery]
    bufferOffset = _serializer.bool(obj.isQuery, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BalanceQueryRequest
    let len;
    let data = new BalanceQueryRequest(null);
    // Deserialize message field [isQuery]
    data.isQuery = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'xr1controllerol/BalanceQueryRequest';
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
    const resolved = new BalanceQueryRequest(null);
    if (msg.isQuery !== undefined) {
      resolved.isQuery = msg.isQuery;
    }
    else {
      resolved.isQuery = false
    }

    return resolved;
    }
};

class BalanceQueryResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.inBLCMode = null;
      this.hasIdle = null;
      this.hasPassive = null;
      this.hasActive = null;
    }
    else {
      if (initObj.hasOwnProperty('inBLCMode')) {
        this.inBLCMode = initObj.inBLCMode
      }
      else {
        this.inBLCMode = false;
      }
      if (initObj.hasOwnProperty('hasIdle')) {
        this.hasIdle = initObj.hasIdle
      }
      else {
        this.hasIdle = false;
      }
      if (initObj.hasOwnProperty('hasPassive')) {
        this.hasPassive = initObj.hasPassive
      }
      else {
        this.hasPassive = false;
      }
      if (initObj.hasOwnProperty('hasActive')) {
        this.hasActive = initObj.hasActive
      }
      else {
        this.hasActive = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BalanceQueryResponse
    // Serialize message field [inBLCMode]
    bufferOffset = _serializer.bool(obj.inBLCMode, buffer, bufferOffset);
    // Serialize message field [hasIdle]
    bufferOffset = _serializer.bool(obj.hasIdle, buffer, bufferOffset);
    // Serialize message field [hasPassive]
    bufferOffset = _serializer.bool(obj.hasPassive, buffer, bufferOffset);
    // Serialize message field [hasActive]
    bufferOffset = _serializer.bool(obj.hasActive, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BalanceQueryResponse
    let len;
    let data = new BalanceQueryResponse(null);
    // Deserialize message field [inBLCMode]
    data.inBLCMode = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [hasIdle]
    data.hasIdle = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [hasPassive]
    data.hasPassive = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [hasActive]
    data.hasActive = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'xr1controllerol/BalanceQueryResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '64124d4b66b6ff17093b36c682f8a41e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool inBLCMode
    bool hasIdle
    bool hasPassive
    bool hasActive
    
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new BalanceQueryResponse(null);
    if (msg.inBLCMode !== undefined) {
      resolved.inBLCMode = msg.inBLCMode;
    }
    else {
      resolved.inBLCMode = false
    }

    if (msg.hasIdle !== undefined) {
      resolved.hasIdle = msg.hasIdle;
    }
    else {
      resolved.hasIdle = false
    }

    if (msg.hasPassive !== undefined) {
      resolved.hasPassive = msg.hasPassive;
    }
    else {
      resolved.hasPassive = false
    }

    if (msg.hasActive !== undefined) {
      resolved.hasActive = msg.hasActive;
    }
    else {
      resolved.hasActive = false
    }

    return resolved;
    }
};

module.exports = {
  Request: BalanceQueryRequest,
  Response: BalanceQueryResponse,
  md5sum() { return 'cd5d7de45f0cd959837e8340a01d26a2'; },
  datatype() { return 'xr1controllerol/BalanceQuery'; }
};
