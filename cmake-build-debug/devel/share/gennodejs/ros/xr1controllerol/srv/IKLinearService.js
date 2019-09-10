// Auto-generated. Do not edit!

// (in-package xr1controllerol.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class IKLinearServiceRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.NewTarget = null;
      this.ControlGroup = null;
      this.BaseGroup = null;
      this.Period = null;
      this.TargetTransform = null;
      this.TargetElbowAngle = null;
      this.Grip = null;
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
      if (initObj.hasOwnProperty('BaseGroup')) {
        this.BaseGroup = initObj.BaseGroup
      }
      else {
        this.BaseGroup = 0;
      }
      if (initObj.hasOwnProperty('Period')) {
        this.Period = initObj.Period
      }
      else {
        this.Period = 0.0;
      }
      if (initObj.hasOwnProperty('TargetTransform')) {
        this.TargetTransform = initObj.TargetTransform
      }
      else {
        this.TargetTransform = new geometry_msgs.msg.Transform();
      }
      if (initObj.hasOwnProperty('TargetElbowAngle')) {
        this.TargetElbowAngle = initObj.TargetElbowAngle
      }
      else {
        this.TargetElbowAngle = 0.0;
      }
      if (initObj.hasOwnProperty('Grip')) {
        this.Grip = initObj.Grip
      }
      else {
        this.Grip = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type IKLinearServiceRequest
    // Serialize message field [NewTarget]
    bufferOffset = _serializer.bool(obj.NewTarget, buffer, bufferOffset);
    // Serialize message field [ControlGroup]
    bufferOffset = _serializer.int32(obj.ControlGroup, buffer, bufferOffset);
    // Serialize message field [BaseGroup]
    bufferOffset = _serializer.int32(obj.BaseGroup, buffer, bufferOffset);
    // Serialize message field [Period]
    bufferOffset = _serializer.float64(obj.Period, buffer, bufferOffset);
    // Serialize message field [TargetTransform]
    bufferOffset = geometry_msgs.msg.Transform.serialize(obj.TargetTransform, buffer, bufferOffset);
    // Serialize message field [TargetElbowAngle]
    bufferOffset = _serializer.float64(obj.TargetElbowAngle, buffer, bufferOffset);
    // Serialize message field [Grip]
    bufferOffset = _serializer.bool(obj.Grip, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type IKLinearServiceRequest
    let len;
    let data = new IKLinearServiceRequest(null);
    // Deserialize message field [NewTarget]
    data.NewTarget = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [ControlGroup]
    data.ControlGroup = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [BaseGroup]
    data.BaseGroup = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [Period]
    data.Period = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [TargetTransform]
    data.TargetTransform = geometry_msgs.msg.Transform.deserialize(buffer, bufferOffset);
    // Deserialize message field [TargetElbowAngle]
    data.TargetElbowAngle = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Grip]
    data.Grip = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 82;
  }

  static datatype() {
    // Returns string type for a service object
    return 'xr1controllerol/IKLinearServiceRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '54bfe6c05ff54abf03bc1d3a3df4ccb0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool NewTarget
    int32 ControlGroup
    int32 BaseGroup
    float64 Period
    geometry_msgs/Transform TargetTransform
    float64 TargetElbowAngle
    bool Grip
    
    ================================================================================
    MSG: geometry_msgs/Transform
    # This represents the transform between two coordinate frames in free space.
    
    Vector3 translation
    Quaternion rotation
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new IKLinearServiceRequest(null);
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

    if (msg.BaseGroup !== undefined) {
      resolved.BaseGroup = msg.BaseGroup;
    }
    else {
      resolved.BaseGroup = 0
    }

    if (msg.Period !== undefined) {
      resolved.Period = msg.Period;
    }
    else {
      resolved.Period = 0.0
    }

    if (msg.TargetTransform !== undefined) {
      resolved.TargetTransform = geometry_msgs.msg.Transform.Resolve(msg.TargetTransform)
    }
    else {
      resolved.TargetTransform = new geometry_msgs.msg.Transform()
    }

    if (msg.TargetElbowAngle !== undefined) {
      resolved.TargetElbowAngle = msg.TargetElbowAngle;
    }
    else {
      resolved.TargetElbowAngle = 0.0
    }

    if (msg.Grip !== undefined) {
      resolved.Grip = msg.Grip;
    }
    else {
      resolved.Grip = false
    }

    return resolved;
    }
};

class IKLinearServiceResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.inProgress = null;
      this.isReachable = null;
      this.isAccepted = null;
    }
    else {
      if (initObj.hasOwnProperty('inProgress')) {
        this.inProgress = initObj.inProgress
      }
      else {
        this.inProgress = false;
      }
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
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type IKLinearServiceResponse
    // Serialize message field [inProgress]
    bufferOffset = _serializer.bool(obj.inProgress, buffer, bufferOffset);
    // Serialize message field [isReachable]
    bufferOffset = _serializer.bool(obj.isReachable, buffer, bufferOffset);
    // Serialize message field [isAccepted]
    bufferOffset = _serializer.bool(obj.isAccepted, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type IKLinearServiceResponse
    let len;
    let data = new IKLinearServiceResponse(null);
    // Deserialize message field [inProgress]
    data.inProgress = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [isReachable]
    data.isReachable = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [isAccepted]
    data.isAccepted = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 3;
  }

  static datatype() {
    // Returns string type for a service object
    return 'xr1controllerol/IKLinearServiceResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'aafe42d2bbfb9a8bb25a5cc12975e8f6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool inProgress
    bool isReachable
    bool isAccepted
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new IKLinearServiceResponse(null);
    if (msg.inProgress !== undefined) {
      resolved.inProgress = msg.inProgress;
    }
    else {
      resolved.inProgress = false
    }

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

    return resolved;
    }
};

module.exports = {
  Request: IKLinearServiceRequest,
  Response: IKLinearServiceResponse,
  md5sum() { return '396edb051773c9bb790828120442568f'; },
  datatype() { return 'xr1controllerol/IKLinearService'; }
};
