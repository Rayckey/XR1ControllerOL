; Auto-generated. Do not edit!


(cl:in-package xr1controllerol-srv)


;//! \htmlinclude AnimationOverwrite-request.msg.html

(cl:defclass <AnimationOverwrite-request> (roslisp-msg-protocol:ros-message)
  ((AnimationType
    :reader AnimationType
    :initarg :AnimationType
    :type cl:integer
    :initform 0)
   (AnimationID
    :reader AnimationID
    :initarg :AnimationID
    :type cl:integer
    :initform 0)
   (AnimationData
    :reader AnimationData
    :initarg :AnimationData
    :type std_msgs-msg:Float64MultiArray
    :initform (cl:make-instance 'std_msgs-msg:Float64MultiArray)))
)

(cl:defclass AnimationOverwrite-request (<AnimationOverwrite-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AnimationOverwrite-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AnimationOverwrite-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xr1controllerol-srv:<AnimationOverwrite-request> is deprecated: use xr1controllerol-srv:AnimationOverwrite-request instead.")))

(cl:ensure-generic-function 'AnimationType-val :lambda-list '(m))
(cl:defmethod AnimationType-val ((m <AnimationOverwrite-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:AnimationType-val is deprecated.  Use xr1controllerol-srv:AnimationType instead.")
  (AnimationType m))

(cl:ensure-generic-function 'AnimationID-val :lambda-list '(m))
(cl:defmethod AnimationID-val ((m <AnimationOverwrite-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:AnimationID-val is deprecated.  Use xr1controllerol-srv:AnimationID instead.")
  (AnimationID m))

(cl:ensure-generic-function 'AnimationData-val :lambda-list '(m))
(cl:defmethod AnimationData-val ((m <AnimationOverwrite-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:AnimationData-val is deprecated.  Use xr1controllerol-srv:AnimationData instead.")
  (AnimationData m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AnimationOverwrite-request>) ostream)
  "Serializes a message object of type '<AnimationOverwrite-request>"
  (cl:let* ((signed (cl:slot-value msg 'AnimationType)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
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
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'AnimationData) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AnimationOverwrite-request>) istream)
  "Deserializes a message object of type '<AnimationOverwrite-request>"
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
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'AnimationID) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'AnimationData) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AnimationOverwrite-request>)))
  "Returns string type for a service object of type '<AnimationOverwrite-request>"
  "xr1controllerol/AnimationOverwriteRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AnimationOverwrite-request)))
  "Returns string type for a service object of type 'AnimationOverwrite-request"
  "xr1controllerol/AnimationOverwriteRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AnimationOverwrite-request>)))
  "Returns md5sum for a message object of type '<AnimationOverwrite-request>"
  "7243db66331b06264c2b466bb4f193aa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AnimationOverwrite-request)))
  "Returns md5sum for a message object of type 'AnimationOverwrite-request"
  "7243db66331b06264c2b466bb4f193aa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AnimationOverwrite-request>)))
  "Returns full string definition for message of type '<AnimationOverwrite-request>"
  (cl:format cl:nil "int32 AnimationType~%int64 AnimationID~%std_msgs/Float64MultiArray AnimationData~%~%================================================================================~%MSG: std_msgs/Float64MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float64[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AnimationOverwrite-request)))
  "Returns full string definition for message of type 'AnimationOverwrite-request"
  (cl:format cl:nil "int32 AnimationType~%int64 AnimationID~%std_msgs/Float64MultiArray AnimationData~%~%================================================================================~%MSG: std_msgs/Float64MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float64[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AnimationOverwrite-request>))
  (cl:+ 0
     4
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'AnimationData))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AnimationOverwrite-request>))
  "Converts a ROS message object to a list"
  (cl:list 'AnimationOverwrite-request
    (cl:cons ':AnimationType (AnimationType msg))
    (cl:cons ':AnimationID (AnimationID msg))
    (cl:cons ':AnimationData (AnimationData msg))
))
;//! \htmlinclude AnimationOverwrite-response.msg.html

(cl:defclass <AnimationOverwrite-response> (roslisp-msg-protocol:ros-message)
  ((isLoaded
    :reader isLoaded
    :initarg :isLoaded
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass AnimationOverwrite-response (<AnimationOverwrite-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AnimationOverwrite-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AnimationOverwrite-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xr1controllerol-srv:<AnimationOverwrite-response> is deprecated: use xr1controllerol-srv:AnimationOverwrite-response instead.")))

(cl:ensure-generic-function 'isLoaded-val :lambda-list '(m))
(cl:defmethod isLoaded-val ((m <AnimationOverwrite-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:isLoaded-val is deprecated.  Use xr1controllerol-srv:isLoaded instead.")
  (isLoaded m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AnimationOverwrite-response>) ostream)
  "Serializes a message object of type '<AnimationOverwrite-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'isLoaded) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AnimationOverwrite-response>) istream)
  "Deserializes a message object of type '<AnimationOverwrite-response>"
    (cl:setf (cl:slot-value msg 'isLoaded) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AnimationOverwrite-response>)))
  "Returns string type for a service object of type '<AnimationOverwrite-response>"
  "xr1controllerol/AnimationOverwriteResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AnimationOverwrite-response)))
  "Returns string type for a service object of type 'AnimationOverwrite-response"
  "xr1controllerol/AnimationOverwriteResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AnimationOverwrite-response>)))
  "Returns md5sum for a message object of type '<AnimationOverwrite-response>"
  "7243db66331b06264c2b466bb4f193aa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AnimationOverwrite-response)))
  "Returns md5sum for a message object of type 'AnimationOverwrite-response"
  "7243db66331b06264c2b466bb4f193aa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AnimationOverwrite-response>)))
  "Returns full string definition for message of type '<AnimationOverwrite-response>"
  (cl:format cl:nil "bool isLoaded~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AnimationOverwrite-response)))
  "Returns full string definition for message of type 'AnimationOverwrite-response"
  (cl:format cl:nil "bool isLoaded~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AnimationOverwrite-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AnimationOverwrite-response>))
  "Converts a ROS message object to a list"
  (cl:list 'AnimationOverwrite-response
    (cl:cons ':isLoaded (isLoaded msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'AnimationOverwrite)))
  'AnimationOverwrite-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'AnimationOverwrite)))
  'AnimationOverwrite-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AnimationOverwrite)))
  "Returns string type for a service object of type '<AnimationOverwrite>"
  "xr1controllerol/AnimationOverwrite")