; Auto-generated. Do not edit!


(cl:in-package xr1controllerol-srv)


;//! \htmlinclude IKTrackingService-request.msg.html

(cl:defclass <IKTrackingService-request> (roslisp-msg-protocol:ros-message)
  ((NewTarget
    :reader NewTarget
    :initarg :NewTarget
    :type cl:boolean
    :initform cl:nil)
   (ControlGroup
    :reader ControlGroup
    :initarg :ControlGroup
    :type cl:integer
    :initform 0))
)

(cl:defclass IKTrackingService-request (<IKTrackingService-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <IKTrackingService-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'IKTrackingService-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xr1controllerol-srv:<IKTrackingService-request> is deprecated: use xr1controllerol-srv:IKTrackingService-request instead.")))

(cl:ensure-generic-function 'NewTarget-val :lambda-list '(m))
(cl:defmethod NewTarget-val ((m <IKTrackingService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:NewTarget-val is deprecated.  Use xr1controllerol-srv:NewTarget instead.")
  (NewTarget m))

(cl:ensure-generic-function 'ControlGroup-val :lambda-list '(m))
(cl:defmethod ControlGroup-val ((m <IKTrackingService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:ControlGroup-val is deprecated.  Use xr1controllerol-srv:ControlGroup instead.")
  (ControlGroup m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <IKTrackingService-request>) ostream)
  "Serializes a message object of type '<IKTrackingService-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'NewTarget) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'ControlGroup)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <IKTrackingService-request>) istream)
  "Deserializes a message object of type '<IKTrackingService-request>"
    (cl:setf (cl:slot-value msg 'NewTarget) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ControlGroup) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<IKTrackingService-request>)))
  "Returns string type for a service object of type '<IKTrackingService-request>"
  "xr1controllerol/IKTrackingServiceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IKTrackingService-request)))
  "Returns string type for a service object of type 'IKTrackingService-request"
  "xr1controllerol/IKTrackingServiceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<IKTrackingService-request>)))
  "Returns md5sum for a message object of type '<IKTrackingService-request>"
  "c77f0c416d0a5b9b08f361d079852e35")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'IKTrackingService-request)))
  "Returns md5sum for a message object of type 'IKTrackingService-request"
  "c77f0c416d0a5b9b08f361d079852e35")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<IKTrackingService-request>)))
  "Returns full string definition for message of type '<IKTrackingService-request>"
  (cl:format cl:nil "bool NewTarget~%int32 ControlGroup~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'IKTrackingService-request)))
  "Returns full string definition for message of type 'IKTrackingService-request"
  (cl:format cl:nil "bool NewTarget~%int32 ControlGroup~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <IKTrackingService-request>))
  (cl:+ 0
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <IKTrackingService-request>))
  "Converts a ROS message object to a list"
  (cl:list 'IKTrackingService-request
    (cl:cons ':NewTarget (NewTarget msg))
    (cl:cons ':ControlGroup (ControlGroup msg))
))
;//! \htmlinclude IKTrackingService-response.msg.html

(cl:defclass <IKTrackingService-response> (roslisp-msg-protocol:ros-message)
  ((isReachable
    :reader isReachable
    :initarg :isReachable
    :type cl:boolean
    :initform cl:nil)
   (isAccepted
    :reader isAccepted
    :initarg :isAccepted
    :type cl:boolean
    :initform cl:nil)
   (inProgress
    :reader inProgress
    :initarg :inProgress
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass IKTrackingService-response (<IKTrackingService-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <IKTrackingService-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'IKTrackingService-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xr1controllerol-srv:<IKTrackingService-response> is deprecated: use xr1controllerol-srv:IKTrackingService-response instead.")))

(cl:ensure-generic-function 'isReachable-val :lambda-list '(m))
(cl:defmethod isReachable-val ((m <IKTrackingService-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:isReachable-val is deprecated.  Use xr1controllerol-srv:isReachable instead.")
  (isReachable m))

(cl:ensure-generic-function 'isAccepted-val :lambda-list '(m))
(cl:defmethod isAccepted-val ((m <IKTrackingService-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:isAccepted-val is deprecated.  Use xr1controllerol-srv:isAccepted instead.")
  (isAccepted m))

(cl:ensure-generic-function 'inProgress-val :lambda-list '(m))
(cl:defmethod inProgress-val ((m <IKTrackingService-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:inProgress-val is deprecated.  Use xr1controllerol-srv:inProgress instead.")
  (inProgress m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <IKTrackingService-response>) ostream)
  "Serializes a message object of type '<IKTrackingService-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'isReachable) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'isAccepted) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'inProgress) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <IKTrackingService-response>) istream)
  "Deserializes a message object of type '<IKTrackingService-response>"
    (cl:setf (cl:slot-value msg 'isReachable) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'isAccepted) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'inProgress) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<IKTrackingService-response>)))
  "Returns string type for a service object of type '<IKTrackingService-response>"
  "xr1controllerol/IKTrackingServiceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IKTrackingService-response)))
  "Returns string type for a service object of type 'IKTrackingService-response"
  "xr1controllerol/IKTrackingServiceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<IKTrackingService-response>)))
  "Returns md5sum for a message object of type '<IKTrackingService-response>"
  "c77f0c416d0a5b9b08f361d079852e35")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'IKTrackingService-response)))
  "Returns md5sum for a message object of type 'IKTrackingService-response"
  "c77f0c416d0a5b9b08f361d079852e35")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<IKTrackingService-response>)))
  "Returns full string definition for message of type '<IKTrackingService-response>"
  (cl:format cl:nil "bool isReachable~%bool isAccepted~%bool inProgress~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'IKTrackingService-response)))
  "Returns full string definition for message of type 'IKTrackingService-response"
  (cl:format cl:nil "bool isReachable~%bool isAccepted~%bool inProgress~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <IKTrackingService-response>))
  (cl:+ 0
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <IKTrackingService-response>))
  "Converts a ROS message object to a list"
  (cl:list 'IKTrackingService-response
    (cl:cons ':isReachable (isReachable msg))
    (cl:cons ':isAccepted (isAccepted msg))
    (cl:cons ':inProgress (inProgress msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'IKTrackingService)))
  'IKTrackingService-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'IKTrackingService)))
  'IKTrackingService-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IKTrackingService)))
  "Returns string type for a service object of type '<IKTrackingService>"
  "xr1controllerol/IKTrackingService")