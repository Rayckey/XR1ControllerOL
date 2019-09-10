; Auto-generated. Do not edit!


(cl:in-package xr1controllerol-msg)


;//! \htmlinclude AnimationMsgs.msg.html

(cl:defclass <AnimationMsgs> (roslisp-msg-protocol:ros-message)
  ((AnimationType
    :reader AnimationType
    :initarg :AnimationType
    :type cl:integer
    :initform 0)
   (AnimationID
    :reader AnimationID
    :initarg :AnimationID
    :type cl:integer
    :initform 0))
)

(cl:defclass AnimationMsgs (<AnimationMsgs>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AnimationMsgs>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AnimationMsgs)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xr1controllerol-msg:<AnimationMsgs> is deprecated: use xr1controllerol-msg:AnimationMsgs instead.")))

(cl:ensure-generic-function 'AnimationType-val :lambda-list '(m))
(cl:defmethod AnimationType-val ((m <AnimationMsgs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-msg:AnimationType-val is deprecated.  Use xr1controllerol-msg:AnimationType instead.")
  (AnimationType m))

(cl:ensure-generic-function 'AnimationID-val :lambda-list '(m))
(cl:defmethod AnimationID-val ((m <AnimationMsgs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-msg:AnimationID-val is deprecated.  Use xr1controllerol-msg:AnimationID instead.")
  (AnimationID m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AnimationMsgs>) ostream)
  "Serializes a message object of type '<AnimationMsgs>"
  (cl:let* ((signed (cl:slot-value msg 'AnimationType)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'AnimationID)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AnimationMsgs>) istream)
  "Deserializes a message object of type '<AnimationMsgs>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'AnimationType) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'AnimationID) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AnimationMsgs>)))
  "Returns string type for a message object of type '<AnimationMsgs>"
  "xr1controllerol/AnimationMsgs")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AnimationMsgs)))
  "Returns string type for a message object of type 'AnimationMsgs"
  "xr1controllerol/AnimationMsgs")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AnimationMsgs>)))
  "Returns md5sum for a message object of type '<AnimationMsgs>"
  "9e4a55e887ede9012b0ab385278da204")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AnimationMsgs)))
  "Returns md5sum for a message object of type 'AnimationMsgs"
  "9e4a55e887ede9012b0ab385278da204")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AnimationMsgs>)))
  "Returns full string definition for message of type '<AnimationMsgs>"
  (cl:format cl:nil "int32 AnimationType~%int32 AnimationID~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AnimationMsgs)))
  "Returns full string definition for message of type 'AnimationMsgs"
  (cl:format cl:nil "int32 AnimationType~%int32 AnimationID~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AnimationMsgs>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AnimationMsgs>))
  "Converts a ROS message object to a list"
  (cl:list 'AnimationMsgs
    (cl:cons ':AnimationType (AnimationType msg))
    (cl:cons ':AnimationID (AnimationID msg))
))
