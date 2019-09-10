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

class RobotStateQueryRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.isQuery = null;
      this.requestLift = null;
    }
    else {
      if (initObj.hasOwnProperty('isQuery')) {
        this.isQuery = initObj.isQuery
      }
      else {
        this.isQuery = false;
      }
      if (initObj.hasOwnProperty('requestLift')) {
        this.requestLift = initObj.requestLift
      }
      else {
        this.requestLift = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RobotStateQueryRequest
    // Serialize message field [isQuery]
    bufferOffset = _serializer.bool(obj.isQuery, buffer, bufferOffset);
    // Serialize message field [requestLift]
    bufferOffset = _serializer.bool(obj.requestLift, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RobotStateQueryRequest
    let len;
    let data = new RobotStateQueryRequest(null);
    // Deserialize message field [isQuery]
    data.isQuery = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [requestLift]
    data.requestLift = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 2;
  }

  static datatype() {
    // Returns string type for a service object
    return 'xr1controllerol/RobotStateQueryRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7b480b0ea9f565f15a42df2dd24b3a26';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool isQuery
    bool requestLift
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RobotStateQueryRequest(null);
    if (msg.isQuery !== undefined) {
      resolved.isQuery = msg.isQuery;
    }
    else {
      resolved.isQuery = false
    }

    if (msg.requestLift !== undefined) {
      resolved.requestLift = msg.requestLift;
    }
    else {
      resolved.requestLift = false
    }

    return resolved;
    }
};

class RobotStateQueryResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.isOkay = null;
      this.RobotState = null;
      this.CollisionSwitch = null;
      this.MainBodyMode = null;
      this.HeadBodyMode = null;
      this.LeftArmMode = null;
      this.RightArmMode = null;
      this.LeftHandMode = null;
      this.RightHandMode = null;
    }
    else {
      if (initObj.hasOwnProperty('isOkay')) {
        this.isOkay = initObj.isOkay
      }
      else {
        this.isOkay = false;
      }
      if (initObj.hasOwnProperty('RobotState')) {
        this.RobotState = initObj.RobotState
      }
      else {
        this.RobotState = 0;
      }
      if (initObj.hasOwnProperty('CollisionSwitch')) {
        this.CollisionSwitch = initObj.CollisionSwitch
      }
      else {
        this.CollisionSwitch = false;
      }
      if (initObj.hasOwnProperty('MainBodyMode')) {
        this.MainBodyMode = initObj.MainBodyMode
      }
      else {
        this.MainBodyMode = 0;
      }
      if (initObj.hasOwnProperty('HeadBodyMode')) {
        this.HeadBodyMode = initObj.HeadBodyMode
      }
      else {
        this.HeadBodyMode = 0;
      }
      if (initObj.hasOwnProperty('LeftArmMode')) {
        this.LeftArmMode = initObj.LeftArmMode
      }
      else {
        this.LeftArmMode = 0;
      }
      if (initObj.hasOwnProperty('RightArmMode')) {
        this.RightArmMode = initObj.RightArmMode
      }
      else {
        this.RightArmMode = 0;
      }
      if (initObj.hasOwnProperty('LeftHandMode')) {
        this.LeftHandMode = initObj.LeftHandMode
      }
      else {
        this.LeftHandMode = 0;
      }
      if (initObj.hasOwnProperty('RightHandMode')) {
        this.RightHandMode = initObj.RightHandMode
      }
      else {
        this.RightHandMode = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RobotStateQueryResponse
    // Serialize message field [isOkay]
    bufferOffset = _serializer.bool(obj.isOkay, buffer, bufferOffset);
    // Serialize message field [RobotState]
    bufferOffset = _serializer.int16(obj.RobotState, buffer, bufferOffset);
    // Serialize message field [CollisionSwitch]
    bufferOffset = _serializer.bool(obj.CollisionSwitch, buffer, bufferOffset);
    // Serialize message field [MainBodyMode]
    bufferOffset = _serializer.int16(obj.MainBodyMode, buffer, bufferOffset);
    // Serialize message field [HeadBodyMode]
    bufferOffset = _serializer.int16(obj.HeadBodyMode, buffer, bufferOffset);
    // Serialize message field [LeftArmMode]
    bufferOffset = _serializer.int16(obj.LeftArmMode, buffer, bufferOffset);
    // Serialize message field [RightArmMode]
    bufferOffset = _serializer.int16(obj.RightArmMode, buffer, bufferOffset);
    // Serialize message field [LeftHandMode]
    bufferOffset = _serializer.int16(obj.LeftHandMode, buffer, bufferOffset);
    // Serialize message field [RightHandMode]
    bufferOffset = _serializer.int16(obj.RightHandMode, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RobotStateQueryResponse
    let len;
    let data = new RobotStateQueryResponse(null);
    // Deserialize message field [isOkay]
    data.isOkay = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [RobotState]
    data.RobotState = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [CollisionSwitch]
    data.CollisionSwitch = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [MainBodyMode]
    data.MainBodyMode = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [HeadBodyMode]
    data.HeadBodyMode = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [LeftArmMode]
    data.LeftArmMode = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [RightArmMode]
    data.RightArmMode = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [LeftHandMode]
    data.LeftHandMode = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [RightHandMode]
    data.RightHandMode = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a service object
    return 'xr1controllerol/RobotStateQueryResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5794dfa17d8da414008d6fe2af401184';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool isOkay
    int16 RobotState
    bool CollisionSwitch
    int16 MainBodyMode
    int16 HeadBodyMode
    int16 LeftArmMode
    int16 RightArmMode
    int16 LeftHandMode
    int16 RightHandMode
    
    
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RobotStateQueryResponse(null);
    if (msg.isOkay !== undefined) {
      resolved.isOkay = msg.isOkay;
    }
    else {
      resolved.isOkay = false
    }

    if (msg.RobotState !== undefined) {
      resolved.RobotState = msg.RobotState;
    }
    else {
      resolved.RobotState = 0
    }

    if (msg.CollisionSwitch !== undefined) {
      resolved.CollisionSwitch = msg.CollisionSwitch;
    }
    else {
      resolved.CollisionSwitch = false
    }

    if (msg.MainBodyMode !== undefined) {
      resolved.MainBodyMode = msg.MainBodyMode;
    }
    else {
      resolved.MainBodyMode = 0
    }

    if (msg.HeadBodyMode !== undefined) {
      resolved.HeadBodyMode = msg.HeadBodyMode;
    }
    else {
      resolved.HeadBodyMode = 0
    }

    if (msg.LeftArmMode !== undefined) {
      resolved.LeftArmMode = msg.LeftArmMode;
    }
    else {
      resolved.LeftArmMode = 0
    }

    if (msg.RightArmMode !== undefined) {
      resolved.RightArmMode = msg.RightArmMode;
    }
    else {
      resolved.RightArmMode = 0
    }

    if (msg.LeftHandMode !== undefined) {
      resolved.LeftHandMode = msg.LeftHandMode;
    }
    else {
      resolved.LeftHandMode = 0
    }

    if (msg.RightHandMode !== undefined) {
      resolved.RightHandMode = msg.RightHandMode;
    }
    else {
      resolved.RightHandMode = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: RobotStateQueryRequest,
  Response: RobotStateQueryResponse,
  md5sum() { return '7e1984b1a9b270399515e3ff3569d4d2'; },
  datatype() { return 'xr1controllerol/RobotStateQuery'; }
};
