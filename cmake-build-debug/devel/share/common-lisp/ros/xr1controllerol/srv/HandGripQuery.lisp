; Auto-generated. Do not edit!


(cl:in-package xr1controllerol-srv)


;//! \htmlinclude HandGripQuery-request.msg.html

(cl:defclass <HandGripQuery-request> (roslisp-msg-protocol:ros-message)
  ((ControlGroup
    :reader ControlGroup
    :initarg :ControlGroup
    :type cl:integer
    :initform 0))
)

(cl:defclass HandGripQuery-request (<HandGripQuery-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HandGripQuery-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HandGripQuery-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xr1controllerol-srv:<HandGripQuery-request> is deprecated: use xr1controllerol-srv:HandGripQuery-request instead.")))

(cl:ensure-generic-function 'ControlGroup-val :lambda-list '(m))
(cl:defmethod ControlGroup-val ((m <HandGripQuery-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:ControlGroup-val is deprecated.  Use xr1controllerol-srv:ControlGroup instead.")
  (ControlGroup m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HandGripQuery-request>) ostream)
  "Serializes a message object of type '<HandGripQuery-request>"
  (cl:let* ((signed (cl:slot-value msg 'ControlGroup)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HandGripQuery-request>) istream)
  "Deserializes a message object of type '<HandGripQuery-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ControlGroup) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HandGripQuery-request>)))
  "Returns string type for a service object of type '<HandGripQuery-request>"
  "xr1controllerol/HandGripQueryRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HandGripQuery-request)))
  "Returns string type for a service object of type 'HandGripQuery-request"
  "xr1controllerol/HandGripQueryRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HandGripQuery-request>)))
  "Returns md5sum for a message object of type '<HandGripQuery-request>"
  "6779f448455fbe1c7c9fa9fca9457608")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HandGripQuery-request)))
  "Returns md5sum for a message object of type 'HandGripQuery-request"
  "6779f448455fbe1c7c9fa9fca9457608")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HandGripQuery-request>)))
  "Returns full string definition for message of type '<HandGripQuery-request>"
  (cl:format cl:nil "int32 ControlGroup~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HandGripQuery-request)))
  "Returns full string definition for message of type 'HandGripQuery-request"
  (cl:format cl:nil "int32 ControlGroup~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HandGripQuery-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HandGripQuery-request>))
  "Converts a ROS message object to a list"
  (cl:list 'HandGripQuery-request
    (cl:cons ':ControlGroup (ControlGroup msg))
))
;//! \htmlinclude HandGripQuery-response.msg.html

(cl:defclass <HandGripQuery-response> (roslisp-msg-protocol:ros-message)
  ((inProgress
    :reader inProgress
    :initarg :inProgress
    :type cl:boolean
    :initform cl:nil)
   (isGripped
    :reader isGripped
    :initarg :isGripped
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass HandGripQuery-response (<HandGripQuery-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HandGripQuery-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HandGripQuery-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xr1controllerol-srv:<HandGripQuery-response> is deprecated: use xr1controllerol-srv:HandGripQuery-response instead.")))

(cl:ensure-generic-function 'inProgress-val :lambda-list '(m))
(cl:defmethod inProgress-val ((m <HandGripQuery-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:inProgress-val is deprecated.  Use xr1controllerol-srv:inProgress instead.")
  (inProgress m))

(cl:ensure-generic-function 'isGripped-val :lambda-list '(m))
(cl:defmethod isGripped-val ((m <HandGripQuery-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:isGripped-val is deprecated.  Use xr1controllerol-srv:isGripped instead.")
  (isGripped m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HandGripQuery-response>) ostream)
  "Serializes a message object of type '<HandGripQuery-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'inProgress) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'isGripped) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HandGripQuery-response>) istream)
  "Deserializes a message object of type '<HandGripQuery-response>"
    (cl:setf (cl:slot-value msg 'inProgress) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'isGripped) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HandGripQuery-response>)))
  "Returns string type for a service object of type '<HandGripQuery-response>"
  "xr1controllerol/HandGripQueryResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HandGripQuery-response)))
  "Returns string type for a service object of type 'HandGripQuery-response"
  "xr1controllerol/HandGripQueryResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HandGripQuery-response>)))
  "Returns md5sum for a message object of type '<HandGripQuery-response>"
  "6779f448455fbe1c7c9fa9fca9457608")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HandGripQuery-response)))
  "Returns md5sum for a message object of type 'HandGripQuery-response"
  "6779f448455fbe1c7c9fa9fca9457608")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HandGripQuery-response>)))
  "Returns full string definition for message of type '<HandGripQuery-response>"
  (cl:format cl:nil "bool inProgress~%bool isGripped~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HandGripQuery-response)))
  "Returns full string definition for message of type 'HandGripQuery-response"
  (cl:format cl:nil "bool inProgress~%bool isGripped~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HandGripQuery-response>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HandGripQuery-response>))
  "Converts a ROS message object to a list"
  (cl:list 'HandGripQuery-response
    (cl:cons ':inProgress (inProgress msg))
    (cl:cons ':isGripped (isGripped msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'HandGripQuery)))
  'HandGripQuery-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'HandGripQuery)))
  'HandGripQuery-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HandGripQuery)))
  "Returns string type for a service object of type '<HandGripQuery>"
  "xr1controllerol/HandGripQuery")