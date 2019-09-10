; Auto-generated. Do not edit!


(cl:in-package xr1controllerol-srv)


;//! \htmlinclude AnimationQuery-request.msg.html

(cl:defclass <AnimationQuery-request> (roslisp-msg-protocol:ros-message)
  ((isQuery
    :reader isQuery
    :initarg :isQuery
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass AnimationQuery-request (<AnimationQuery-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AnimationQuery-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AnimationQuery-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xr1controllerol-srv:<AnimationQuery-request> is deprecated: use xr1controllerol-srv:AnimationQuery-request instead.")))

(cl:ensure-generic-function 'isQuery-val :lambda-list '(m))
(cl:defmethod isQuery-val ((m <AnimationQuery-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:isQuery-val is deprecated.  Use xr1controllerol-srv:isQuery instead.")
  (isQuery m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AnimationQuery-request>) ostream)
  "Serializes a message object of type '<AnimationQuery-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'isQuery) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AnimationQuery-request>) istream)
  "Deserializes a message object of type '<AnimationQuery-request>"
    (cl:setf (cl:slot-value msg 'isQuery) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AnimationQuery-request>)))
  "Returns string type for a service object of type '<AnimationQuery-request>"
  "xr1controllerol/AnimationQueryRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AnimationQuery-request)))
  "Returns string type for a service object of type 'AnimationQuery-request"
  "xr1controllerol/AnimationQueryRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AnimationQuery-request>)))
  "Returns md5sum for a message object of type '<AnimationQuery-request>"
  "5b670fef922a4d6388f8dea2bff6ef00")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AnimationQuery-request)))
  "Returns md5sum for a message object of type 'AnimationQuery-request"
  "5b670fef922a4d6388f8dea2bff6ef00")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AnimationQuery-request>)))
  "Returns full string definition for message of type '<AnimationQuery-request>"
  (cl:format cl:nil "bool isQuery~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AnimationQuery-request)))
  "Returns full string definition for message of type 'AnimationQuery-request"
  (cl:format cl:nil "bool isQuery~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AnimationQuery-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AnimationQuery-request>))
  "Converts a ROS message object to a list"
  (cl:list 'AnimationQuery-request
    (cl:cons ':isQuery (isQuery msg))
))
;//! \htmlinclude AnimationQuery-response.msg.html

(cl:defclass <AnimationQuery-response> (roslisp-msg-protocol:ros-message)
  ((inAnimationMode
    :reader inAnimationMode
    :initarg :inAnimationMode
    :type cl:boolean
    :initform cl:nil)
   (isPlaying
    :reader isPlaying
    :initarg :isPlaying
    :type cl:boolean
    :initform cl:nil)
   (hasDefault
    :reader hasDefault
    :initarg :hasDefault
    :type cl:boolean
    :initform cl:nil)
   (hasIdle
    :reader hasIdle
    :initarg :hasIdle
    :type cl:boolean
    :initform cl:nil)
   (AnimationType
    :reader AnimationType
    :initarg :AnimationType
    :type cl:integer
    :initform 0)
   (AnimationID
    :reader AnimationID
    :initarg :AnimationID
    :type cl:integer
    :initform 0)
   (AnimationProgress
    :reader AnimationProgress
    :initarg :AnimationProgress
    :type cl:integer
    :initform 0))
)

(cl:defclass AnimationQuery-response (<AnimationQuery-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AnimationQuery-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AnimationQuery-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xr1controllerol-srv:<AnimationQuery-response> is deprecated: use xr1controllerol-srv:AnimationQuery-response instead.")))

(cl:ensure-generic-function 'inAnimationMode-val :lambda-list '(m))
(cl:defmethod inAnimationMode-val ((m <AnimationQuery-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:inAnimationMode-val is deprecated.  Use xr1controllerol-srv:inAnimationMode instead.")
  (inAnimationMode m))

(cl:ensure-generic-function 'isPlaying-val :lambda-list '(m))
(cl:defmethod isPlaying-val ((m <AnimationQuery-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:isPlaying-val is deprecated.  Use xr1controllerol-srv:isPlaying instead.")
  (isPlaying m))

(cl:ensure-generic-function 'hasDefault-val :lambda-list '(m))
(cl:defmethod hasDefault-val ((m <AnimationQuery-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:hasDefault-val is deprecated.  Use xr1controllerol-srv:hasDefault instead.")
  (hasDefault m))

(cl:ensure-generic-function 'hasIdle-val :lambda-list '(m))
(cl:defmethod hasIdle-val ((m <AnimationQuery-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:hasIdle-val is deprecated.  Use xr1controllerol-srv:hasIdle instead.")
  (hasIdle m))

(cl:ensure-generic-function 'AnimationType-val :lambda-list '(m))
(cl:defmethod AnimationType-val ((m <AnimationQuery-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:AnimationType-val is deprecated.  Use xr1controllerol-srv:AnimationType instead.")
  (AnimationType m))

(cl:ensure-generic-function 'AnimationID-val :lambda-list '(m))
(cl:defmethod AnimationID-val ((m <AnimationQuery-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:AnimationID-val is deprecated.  Use xr1controllerol-srv:AnimationID instead.")
  (AnimationID m))

(cl:ensure-generic-function 'AnimationProgress-val :lambda-list '(m))
(cl:defmethod AnimationProgress-val ((m <AnimationQuery-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:AnimationProgress-val is deprecated.  Use xr1controllerol-srv:AnimationProgress instead.")
  (AnimationProgress m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AnimationQuery-response>) ostream)
  "Serializes a message object of type '<AnimationQuery-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'inAnimationMode) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'isPlaying) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'hasDefault) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'hasIdle) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'AnimationType)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'AnimationID)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'AnimationProgress)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AnimationQuery-response>) istream)
  "Deserializes a message object of type '<AnimationQuery-response>"
    (cl:setf (cl:slot-value msg 'inAnimationMode) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'isPlaying) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'hasDefault) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'hasIdle) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'AnimationType) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'AnimationID) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'AnimationProgress) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AnimationQuery-response>)))
  "Returns string type for a service object of type '<AnimationQuery-response>"
  "xr1controllerol/AnimationQueryResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AnimationQuery-response)))
  "Returns string type for a service object of type 'AnimationQuery-response"
  "xr1controllerol/AnimationQueryResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AnimationQuery-response>)))
  "Returns md5sum for a message object of type '<AnimationQuery-response>"
  "5b670fef922a4d6388f8dea2bff6ef00")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AnimationQuery-response)))
  "Returns md5sum for a message object of type 'AnimationQuery-response"
  "5b670fef922a4d6388f8dea2bff6ef00")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AnimationQuery-response>)))
  "Returns full string definition for message of type '<AnimationQuery-response>"
  (cl:format cl:nil "bool inAnimationMode~%bool isPlaying~%bool hasDefault~%bool hasIdle~%int64 AnimationType~%int64 AnimationID~%int64 AnimationProgress~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AnimationQuery-response)))
  "Returns full string definition for message of type 'AnimationQuery-response"
  (cl:format cl:nil "bool inAnimationMode~%bool isPlaying~%bool hasDefault~%bool hasIdle~%int64 AnimationType~%int64 AnimationID~%int64 AnimationProgress~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AnimationQuery-response>))
  (cl:+ 0
     1
     1
     1
     1
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AnimationQuery-response>))
  "Converts a ROS message object to a list"
  (cl:list 'AnimationQuery-response
    (cl:cons ':inAnimationMode (inAnimationMode msg))
    (cl:cons ':isPlaying (isPlaying msg))
    (cl:cons ':hasDefault (hasDefault msg))
    (cl:cons ':hasIdle (hasIdle msg))
    (cl:cons ':AnimationType (AnimationType msg))
    (cl:cons ':AnimationID (AnimationID msg))
    (cl:cons ':AnimationProgress (AnimationProgress msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'AnimationQuery)))
  'AnimationQuery-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'AnimationQuery)))
  'AnimationQuery-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AnimationQuery)))
  "Returns string type for a service object of type '<AnimationQuery>"
  "xr1controllerol/AnimationQuery")