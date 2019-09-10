; Auto-generated. Do not edit!


(cl:in-package xr1controllerol-srv)


;//! \htmlinclude BalanceQuery-request.msg.html

(cl:defclass <BalanceQuery-request> (roslisp-msg-protocol:ros-message)
  ((isQuery
    :reader isQuery
    :initarg :isQuery
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass BalanceQuery-request (<BalanceQuery-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BalanceQuery-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BalanceQuery-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xr1controllerol-srv:<BalanceQuery-request> is deprecated: use xr1controllerol-srv:BalanceQuery-request instead.")))

(cl:ensure-generic-function 'isQuery-val :lambda-list '(m))
(cl:defmethod isQuery-val ((m <BalanceQuery-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:isQuery-val is deprecated.  Use xr1controllerol-srv:isQuery instead.")
  (isQuery m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BalanceQuery-request>) ostream)
  "Serializes a message object of type '<BalanceQuery-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'isQuery) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BalanceQuery-request>) istream)
  "Deserializes a message object of type '<BalanceQuery-request>"
    (cl:setf (cl:slot-value msg 'isQuery) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BalanceQuery-request>)))
  "Returns string type for a service object of type '<BalanceQuery-request>"
  "xr1controllerol/BalanceQueryRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BalanceQuery-request)))
  "Returns string type for a service object of type 'BalanceQuery-request"
  "xr1controllerol/BalanceQueryRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BalanceQuery-request>)))
  "Returns md5sum for a message object of type '<BalanceQuery-request>"
  "cd5d7de45f0cd959837e8340a01d26a2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BalanceQuery-request)))
  "Returns md5sum for a message object of type 'BalanceQuery-request"
  "cd5d7de45f0cd959837e8340a01d26a2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BalanceQuery-request>)))
  "Returns full string definition for message of type '<BalanceQuery-request>"
  (cl:format cl:nil "bool isQuery~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BalanceQuery-request)))
  "Returns full string definition for message of type 'BalanceQuery-request"
  (cl:format cl:nil "bool isQuery~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BalanceQuery-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BalanceQuery-request>))
  "Converts a ROS message object to a list"
  (cl:list 'BalanceQuery-request
    (cl:cons ':isQuery (isQuery msg))
))
;//! \htmlinclude BalanceQuery-response.msg.html

(cl:defclass <BalanceQuery-response> (roslisp-msg-protocol:ros-message)
  ((inBLCMode
    :reader inBLCMode
    :initarg :inBLCMode
    :type cl:boolean
    :initform cl:nil)
   (hasIdle
    :reader hasIdle
    :initarg :hasIdle
    :type cl:boolean
    :initform cl:nil)
   (hasPassive
    :reader hasPassive
    :initarg :hasPassive
    :type cl:boolean
    :initform cl:nil)
   (hasActive
    :reader hasActive
    :initarg :hasActive
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass BalanceQuery-response (<BalanceQuery-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BalanceQuery-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BalanceQuery-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xr1controllerol-srv:<BalanceQuery-response> is deprecated: use xr1controllerol-srv:BalanceQuery-response instead.")))

(cl:ensure-generic-function 'inBLCMode-val :lambda-list '(m))
(cl:defmethod inBLCMode-val ((m <BalanceQuery-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:inBLCMode-val is deprecated.  Use xr1controllerol-srv:inBLCMode instead.")
  (inBLCMode m))

(cl:ensure-generic-function 'hasIdle-val :lambda-list '(m))
(cl:defmethod hasIdle-val ((m <BalanceQuery-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:hasIdle-val is deprecated.  Use xr1controllerol-srv:hasIdle instead.")
  (hasIdle m))

(cl:ensure-generic-function 'hasPassive-val :lambda-list '(m))
(cl:defmethod hasPassive-val ((m <BalanceQuery-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:hasPassive-val is deprecated.  Use xr1controllerol-srv:hasPassive instead.")
  (hasPassive m))

(cl:ensure-generic-function 'hasActive-val :lambda-list '(m))
(cl:defmethod hasActive-val ((m <BalanceQuery-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:hasActive-val is deprecated.  Use xr1controllerol-srv:hasActive instead.")
  (hasActive m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BalanceQuery-response>) ostream)
  "Serializes a message object of type '<BalanceQuery-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'inBLCMode) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'hasIdle) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'hasPassive) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'hasActive) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BalanceQuery-response>) istream)
  "Deserializes a message object of type '<BalanceQuery-response>"
    (cl:setf (cl:slot-value msg 'inBLCMode) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'hasIdle) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'hasPassive) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'hasActive) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BalanceQuery-response>)))
  "Returns string type for a service object of type '<BalanceQuery-response>"
  "xr1controllerol/BalanceQueryResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BalanceQuery-response)))
  "Returns string type for a service object of type 'BalanceQuery-response"
  "xr1controllerol/BalanceQueryResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BalanceQuery-response>)))
  "Returns md5sum for a message object of type '<BalanceQuery-response>"
  "cd5d7de45f0cd959837e8340a01d26a2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BalanceQuery-response)))
  "Returns md5sum for a message object of type 'BalanceQuery-response"
  "cd5d7de45f0cd959837e8340a01d26a2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BalanceQuery-response>)))
  "Returns full string definition for message of type '<BalanceQuery-response>"
  (cl:format cl:nil "bool inBLCMode~%bool hasIdle~%bool hasPassive~%bool hasActive~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BalanceQuery-response)))
  "Returns full string definition for message of type 'BalanceQuery-response"
  (cl:format cl:nil "bool inBLCMode~%bool hasIdle~%bool hasPassive~%bool hasActive~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BalanceQuery-response>))
  (cl:+ 0
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BalanceQuery-response>))
  "Converts a ROS message object to a list"
  (cl:list 'BalanceQuery-response
    (cl:cons ':inBLCMode (inBLCMode msg))
    (cl:cons ':hasIdle (hasIdle msg))
    (cl:cons ':hasPassive (hasPassive msg))
    (cl:cons ':hasActive (hasActive msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'BalanceQuery)))
  'BalanceQuery-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'BalanceQuery)))
  'BalanceQuery-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BalanceQuery)))
  "Returns string type for a service object of type '<BalanceQuery>"
  "xr1controllerol/BalanceQuery")