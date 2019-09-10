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

class IKTrackingServiceRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.NewTarget = null;
      this.ControlGroup = null;
    }
    else {
      if (initObj.hasOwnProperty('NewTarget')) {
        this.NewTarget = initObj.NewTarget
      }
      else {
        this.NewTarget = false;
      }
      if (initObj.hasOwnProperty('ControlGroup')) {
        this.ControlGroup = initObj.ControlGroup
      }
      else {
        this.ControlGroup = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type IKTrackingServiceRequest
    // Serialize message field [NewTarget]
    bufferOffset = _serializer.bool(obj.NewTarget, buffer, bufferOffset);
    // Serialize message field [ControlGroup]
    bufferOffset = _serializer.int32(obj.ControlGroup, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type IKTrackingServiceRequest
    let len;
    let data = new IKTrackingServiceRequest(null);
    // Deserialize message field [NewTarget]
    data.NewTarget = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [ControlGroup]
    data.ControlGroup = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'xr1controllerol/IKTrackingServiceRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '727975d5c6744c0e56b1e97bf241aa3e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool NewTarget
    int32 ControlGroup
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new IKTrackingServiceRequest(null);
    if (msg.NewTarget !== undefined) {
      resolved.NewTarget = msg.NewTarget;
    }
    else {
      resolved.NewTarget = false
    }

    if (msg.ControlGroup !== undefined) {
      resolved.ControlGroup = msg.ControlGroup;
    }
    else {
      resolved.ControlGroup = 0
    }

    return resolved;
    }
};

class IKTrackingServiceResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.isReachable = null;
      this.isAccepted = null;
      this.inProgress = null;
    }
    else {
      if (initObj.hasOwnProperty('isReachable')) {
        this.isReachable = initObj.isReachable
      }
      else {
        this.isReachable = false;
      }
      if (initObj.hasOwnProperty('isAccepted')) {
        this.isAccepted = initObj.isAccepted
      }
      else {
        this.isAccepted = false;
      }
      if (initObj.hasOwnProperty('inProgress')) {
        this.inProgress = initObj.inProgress
      }
      else {
        this.inProgress = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type IKTrackingServiceResponse
    // Serialize message field [isReachable]
    bufferOffset = _serializer.bool(obj.isReachable, buffer, bufferOffset);
    // Serialize message field [isAccepted]
    bufferOffset = _serializer.bool(obj.isAccepted, buffer, bufferOffset);
    // Serialize message field [inProgress]
    bufferOffset = _serializer.bool(obj.inProgress, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type IKTrackingServiceResponse
    let len;
    let data = new IKTrackingServiceResponse(null);
    // Deserialize message field [isReachable]
    data.isReachable = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [isAccepted]
    data.isAccepted = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [inProgress]
    data.inProgress = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 3;
  }

  static datatype() {
    // Returns string type for a service object
    return 'xr1controllerol/IKTrackingServiceResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'eb82ae550eaddbf1c13502e7df527444';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool isReachable
    bool isAccepted
    bool inProgress
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new IKTrackingServiceResponse(null);
    if (msg.isReachable !== undefined) {
      resolved.isReachable = msg.isReachable;
    }
    else {
      resolved.isReachable = false
    }

    if (msg.isAccepted !== undefined) {
      resolved.isAccepted = msg.isAccepted;
    }
    else {
      resolved.isAccepted = false
    }

    if (msg.inProgress !== undefined) {
      resolved.inProgress = msg.inProgress;
    }
    else {
      resolved.inProgress = false
    }

    return resolved;
    }
};

module.exports = {
  Request: IKTrackingServiceRequest,
  Response: IKTrackingServiceResponse,
  md5sum() { return 'c77f0c416d0a5b9b08f361d079852e35'; },
  datatype() { return 'xr1controllerol/IKTrackingService'; }
};
