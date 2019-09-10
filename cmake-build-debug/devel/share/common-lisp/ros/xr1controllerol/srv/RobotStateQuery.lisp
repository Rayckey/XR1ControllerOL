; Auto-generated. Do not edit!


(cl:in-package xr1controllerol-srv)


;//! \htmlinclude RobotStateQuery-request.msg.html

(cl:defclass <RobotStateQuery-request> (roslisp-msg-protocol:ros-message)
  ((isQuery
    :reader isQuery
    :initarg :isQuery
    :type cl:boolean
    :initform cl:nil)
   (requestLift
    :reader requestLift
    :initarg :requestLift
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass RobotStateQuery-request (<RobotStateQuery-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotStateQuery-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotStateQuery-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xr1controllerol-srv:<RobotStateQuery-request> is deprecated: use xr1controllerol-srv:RobotStateQuery-request instead.")))

(cl:ensure-generic-function 'isQuery-val :lambda-list '(m))
(cl:defmethod isQuery-val ((m <RobotStateQuery-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:isQuery-val is deprecated.  Use xr1controllerol-srv:isQuery instead.")
  (isQuery m))

(cl:ensure-generic-function 'requestLift-val :lambda-list '(m))
(cl:defmethod requestLift-val ((m <RobotStateQuery-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:requestLift-val is deprecated.  Use xr1controllerol-srv:requestLift instead.")
  (requestLift m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotStateQuery-request>) ostream)
  "Serializes a message object of type '<RobotStateQuery-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'isQuery) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'requestLift) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotStateQuery-request>) istream)
  "Deserializes a message object of type '<RobotStateQuery-request>"
    (cl:setf (cl:slot-value msg 'isQuery) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'requestLift) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotStateQuery-request>)))
  "Returns string type for a service object of type '<RobotStateQuery-request>"
  "xr1controllerol/RobotStateQueryRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotStateQuery-request)))
  "Returns string type for a service object of type 'RobotStateQuery-request"
  "xr1controllerol/RobotStateQueryRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotStateQuery-request>)))
  "Returns md5sum for a message object of type '<RobotStateQuery-request>"
  "7e1984b1a9b270399515e3ff3569d4d2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotStateQuery-request)))
  "Returns md5sum for a message object of type 'RobotStateQuery-request"
  "7e1984b1a9b270399515e3ff3569d4d2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotStateQuery-request>)))
  "Returns full string definition for message of type '<RobotStateQuery-request>"
  (cl:format cl:nil "bool isQuery~%bool requestLift~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotStateQuery-request)))
  "Returns full string definition for message of type 'RobotStateQuery-request"
  (cl:format cl:nil "bool isQuery~%bool requestLift~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotStateQuery-request>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotStateQuery-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotStateQuery-request
    (cl:cons ':isQuery (isQuery msg))
    (cl:cons ':requestLift (requestLift msg))
))
;//! \htmlinclude RobotStateQuery-response.msg.html

(cl:defclass <RobotStateQuery-response> (roslisp-msg-protocol:ros-message)
  ((isOkay
    :reader isOkay
    :initarg :isOkay
    :type cl:boolean
    :initform cl:nil)
   (RobotState
    :reader RobotState
    :initarg :RobotState
    :type cl:fixnum
    :initform 0)
   (CollisionSwitch
    :reader CollisionSwitch
    :initarg :CollisionSwitch
    :type cl:boolean
    :initform cl:nil)
   (MainBodyMode
    :reader MainBodyMode
    :initarg :MainBodyMode
    :type cl:fixnum
    :initform 0)
   (HeadBodyMode
    :reader HeadBodyMode
    :initarg :HeadBodyMode
    :type cl:fixnum
    :initform 0)
   (LeftArmMode
    :reader LeftArmMode
    :initarg :LeftArmMode
    :type cl:fixnum
    :initform 0)
   (RightArmMode
    :reader RightArmMode
    :initarg :RightArmMode
    :type cl:fixnum
    :initform 0)
   (LeftHandMode
    :reader LeftHandMode
    :initarg :LeftHandMode
    :type cl:fixnum
    :initform 0)
   (RightHandMode
    :reader RightHandMode
    :initarg :RightHandMode
    :type cl:fixnum
    :initform 0))
)

(cl:defclass RobotStateQuery-response (<RobotStateQuery-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotStateQuery-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotStateQuery-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xr1controllerol-srv:<RobotStateQuery-response> is deprecated: use xr1controllerol-srv:RobotStateQuery-response instead.")))

(cl:ensure-generic-function 'isOkay-val :lambda-list '(m))
(cl:defmethod isOkay-val ((m <RobotStateQuery-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:isOkay-val is deprecated.  Use xr1controllerol-srv:isOkay instead.")
  (isOkay m))

(cl:ensure-generic-function 'RobotState-val :lambda-list '(m))
(cl:defmethod RobotState-val ((m <RobotStateQuery-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:RobotState-val is deprecated.  Use xr1controllerol-srv:RobotState instead.")
  (RobotState m))

(cl:ensure-generic-function 'CollisionSwitch-val :lambda-list '(m))
(cl:defmethod CollisionSwitch-val ((m <RobotStateQuery-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:CollisionSwitch-val is deprecated.  Use xr1controllerol-srv:CollisionSwitch instead.")
  (CollisionSwitch m))

(cl:ensure-generic-function 'MainBodyMode-val :lambda-list '(m))
(cl:defmethod MainBodyMode-val ((m <RobotStateQuery-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:MainBodyMode-val is deprecated.  Use xr1controllerol-srv:MainBodyMode instead.")
  (MainBodyMode m))

(cl:ensure-generic-function 'HeadBodyMode-val :lambda-list '(m))
(cl:defmethod HeadBodyMode-val ((m <RobotStateQuery-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:HeadBodyMode-val is deprecated.  Use xr1controllerol-srv:HeadBodyMode instead.")
  (HeadBodyMode m))

(cl:ensure-generic-function 'LeftArmMode-val :lambda-list '(m))
(cl:defmethod LeftArmMode-val ((m <RobotStateQuery-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:LeftArmMode-val is deprecated.  Use xr1controllerol-srv:LeftArmMode instead.")
  (LeftArmMode m))

(cl:ensure-generic-function 'RightArmMode-val :lambda-list '(m))
(cl:defmethod RightArmMode-val ((m <RobotStateQuery-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:RightArmMode-val is deprecated.  Use xr1controllerol-srv:RightArmMode instead.")
  (RightArmMode m))

(cl:ensure-generic-function 'LeftHandMode-val :lambda-list '(m))
(cl:defmethod LeftHandMode-val ((m <RobotStateQuery-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:LeftHandMode-val is deprecated.  Use xr1controllerol-srv:LeftHandMode instead.")
  (LeftHandMode m))

(cl:ensure-generic-function 'RightHandMode-val :lambda-list '(m))
(cl:defmethod RightHandMode-val ((m <RobotStateQuery-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:RightHandMode-val is deprecated.  Use xr1controllerol-srv:RightHandMode instead.")
  (RightHandMode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotStateQuery-response>) ostream)
  "Serializes a message object of type '<RobotStateQuery-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'isOkay) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'RobotState)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'CollisionSwitch) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'MainBodyMode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'HeadBodyMode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'LeftArmMode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'RightArmMode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'LeftHandMode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'RightHandMode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotStateQuery-response>) istream)
  "Deserializes a message object of type '<RobotStateQuery-response>"
    (cl:setf (cl:slot-value msg 'isOkay) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'RobotState) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:setf (cl:slot-value msg 'CollisionSwitch) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'MainBodyMode) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'HeadBodyMode) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'LeftArmMode) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'RightArmMode) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'LeftHandMode) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'RightHandMode) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotStateQuery-response>)))
  "Returns string type for a service object of type '<RobotStateQuery-response>"
  "xr1controllerol/RobotStateQueryResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotStateQuery-response)))
  "Returns string type for a service object of type 'RobotStateQuery-response"
  "xr1controllerol/RobotStateQueryResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotStateQuery-response>)))
  "Returns md5sum for a message object of type '<RobotStateQuery-response>"
  "7e1984b1a9b270399515e3ff3569d4d2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotStateQuery-response)))
  "Returns md5sum for a message object of type 'RobotStateQuery-response"
  "7e1984b1a9b270399515e3ff3569d4d2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotStateQuery-response>)))
  "Returns full string definition for message of type '<RobotStateQuery-response>"
  (cl:format cl:nil "bool isOkay~%int16 RobotState~%bool CollisionSwitch~%int16 MainBodyMode~%int16 HeadBodyMode~%int16 LeftArmMode~%int16 RightArmMode~%int16 LeftHandMode~%int16 RightHandMode~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotStateQuery-response)))
  "Returns full string definition for message of type 'RobotStateQuery-response"
  (cl:format cl:nil "bool isOkay~%int16 RobotState~%bool CollisionSwitch~%int16 MainBodyMode~%int16 HeadBodyMode~%int16 LeftArmMode~%int16 RightArmMode~%int16 LeftHandMode~%int16 RightHandMode~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotStateQuery-response>))
  (cl:+ 0
     1
     2
     1
     2
     2
     2
     2
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotStateQuery-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotStateQuery-response
    (cl:cons ':isOkay (isOkay msg))
    (cl:cons ':RobotState (RobotState msg))
    (cl:cons ':CollisionSwitch (CollisionSwitch msg))
    (cl:cons ':MainBodyMode (MainBodyMode msg))
    (cl:cons ':HeadBodyMode (HeadBodyMode msg))
    (cl:cons ':LeftArmMode (LeftArmMode msg))
    (cl:cons ':RightArmMode (RightArmMode msg))
    (cl:cons ':LeftHandMode (LeftHandMode msg))
    (cl:cons ':RightHandMode (RightHandMode msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RobotStateQuery)))
  'RobotStateQuery-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RobotStateQuery)))
  'RobotStateQuery-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotStateQuery)))
  "Returns string type for a service object of type '<RobotStateQuery>"
  "xr1controllerol/RobotStateQuery")