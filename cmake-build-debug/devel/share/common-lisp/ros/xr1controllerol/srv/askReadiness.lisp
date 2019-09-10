; Auto-generated. Do not edit!


(cl:in-package xr1controllerol-srv)


;//! \htmlinclude askReadiness-request.msg.html

(cl:defclass <askReadiness-request> (roslisp-msg-protocol:ros-message)
  ((isAsking
    :reader isAsking
    :initarg :isAsking
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass askReadiness-request (<askReadiness-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <askReadiness-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'askReadiness-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xr1controllerol-srv:<askReadiness-request> is deprecated: use xr1controllerol-srv:askReadiness-request instead.")))

(cl:ensure-generic-function 'isAsking-val :lambda-list '(m))
(cl:defmethod isAsking-val ((m <askReadiness-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:isAsking-val is deprecated.  Use xr1controllerol-srv:isAsking instead.")
  (isAsking m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <askReadiness-request>) ostream)
  "Serializes a message object of type '<askReadiness-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'isAsking) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <askReadiness-request>) istream)
  "Deserializes a message object of type '<askReadiness-request>"
    (cl:setf (cl:slot-value msg 'isAsking) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<askReadiness-request>)))
  "Returns string type for a service object of type '<askReadiness-request>"
  "xr1controllerol/askReadinessRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'askReadiness-request)))
  "Returns string type for a service object of type 'askReadiness-request"
  "xr1controllerol/askReadinessRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<askReadiness-request>)))
  "Returns md5sum for a message object of type '<askReadiness-request>"
  "4a4751acb64765569565cc0a23cfd7be")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'askReadiness-request)))
  "Returns md5sum for a message object of type 'askReadiness-request"
  "4a4751acb64765569565cc0a23cfd7be")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<askReadiness-request>)))
  "Returns full string definition for message of type '<askReadiness-request>"
  (cl:format cl:nil "bool isAsking~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'askReadiness-request)))
  "Returns full string definition for message of type 'askReadiness-request"
  (cl:format cl:nil "bool isAsking~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <askReadiness-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <askReadiness-request>))
  "Converts a ROS message object to a list"
  (cl:list 'askReadiness-request
    (cl:cons ':isAsking (isAsking msg))
))
;//! \htmlinclude askReadiness-response.msg.html

(cl:defclass <askReadiness-response> (roslisp-msg-protocol:ros-message)
  ((isReady
    :reader isReady
    :initarg :isReady
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass askReadiness-response (<askReadiness-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <askReadiness-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'askReadiness-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xr1controllerol-srv:<askReadiness-response> is deprecated: use xr1controllerol-srv:askReadiness-response instead.")))

(cl:ensure-generic-function 'isReady-val :lambda-list '(m))
(cl:defmethod isReady-val ((m <askReadiness-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:isReady-val is deprecated.  Use xr1controllerol-srv:isReady instead.")
  (isReady m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <askReadiness-response>) ostream)
  "Serializes a message object of type '<askReadiness-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'isReady) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <askReadiness-response>) istream)
  "Deserializes a message object of type '<askReadiness-response>"
    (cl:setf (cl:slot-value msg 'isReady) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<askReadiness-response>)))
  "Returns string type for a service object of type '<askReadiness-response>"
  "xr1controllerol/askReadinessResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'askReadiness-response)))
  "Returns string type for a service object of type 'askReadiness-response"
  "xr1controllerol/askReadinessResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<askReadiness-response>)))
  "Returns md5sum for a message object of type '<askReadiness-response>"
  "4a4751acb64765569565cc0a23cfd7be")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'askReadiness-response)))
  "Returns md5sum for a message object of type 'askReadiness-response"
  "4a4751acb64765569565cc0a23cfd7be")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<askReadiness-response>)))
  "Returns full string definition for message of type '<askReadiness-response>"
  (cl:format cl:nil "bool isReady~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'askReadiness-response)))
  "Returns full string definition for message of type 'askReadiness-response"
  (cl:format cl:nil "bool isReady~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <askReadiness-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <askReadiness-response>))
  "Converts a ROS message object to a list"
  (cl:list 'askReadiness-response
    (cl:cons ':isReady (isReady msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'askReadiness)))
  'askReadiness-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'askReadiness)))
  'askReadiness-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'askReadiness)))
  "Returns string type for a service object of type '<askReadiness>"
  "xr1controllerol/askReadiness")