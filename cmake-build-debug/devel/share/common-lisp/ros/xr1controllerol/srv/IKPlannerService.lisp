; Auto-generated. Do not edit!


(cl:in-package xr1controllerol-srv)


;//! \htmlinclude IKPlannerService-request.msg.html

(cl:defclass <IKPlannerService-request> (roslisp-msg-protocol:ros-message)
  ((NewTarget
    :reader NewTarget
    :initarg :NewTarget
    :type cl:boolean
    :initform cl:nil)
   (ControlGroup
    :reader ControlGroup
    :initarg :ControlGroup
    :type cl:integer
    :initform 0)
   (BaseGroup
    :reader BaseGroup
    :initarg :BaseGroup
    :type cl:integer
    :initform 0)
   (Period
    :reader Period
    :initarg :Period
    :type cl:float
    :initform 0.0)
   (PlannerMethod
    :reader PlannerMethod
    :initarg :PlannerMethod
    :type cl:integer
    :initform 0)
   (TargetTransform
    :reader TargetTransform
    :initarg :TargetTransform
    :type geometry_msgs-msg:Transform
    :initform (cl:make-instance 'geometry_msgs-msg:Transform))
   (Reference1
    :reader Reference1
    :initarg :Reference1
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (Reference2
    :reader Reference2
    :initarg :Reference2
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (TargetElbowAngle
    :reader TargetElbowAngle
    :initarg :TargetElbowAngle
    :type cl:float
    :initform 0.0)
   (Grip
    :reader Grip
    :initarg :Grip
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass IKPlannerService-request (<IKPlannerService-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <IKPlannerService-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'IKPlannerService-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xr1controllerol-srv:<IKPlannerService-request> is deprecated: use xr1controllerol-srv:IKPlannerService-request instead.")))

(cl:ensure-generic-function 'NewTarget-val :lambda-list '(m))
(cl:defmethod NewTarget-val ((m <IKPlannerService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:NewTarget-val is deprecated.  Use xr1controllerol-srv:NewTarget instead.")
  (NewTarget m))

(cl:ensure-generic-function 'ControlGroup-val :lambda-list '(m))
(cl:defmethod ControlGroup-val ((m <IKPlannerService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:ControlGroup-val is deprecated.  Use xr1controllerol-srv:ControlGroup instead.")
  (ControlGroup m))

(cl:ensure-generic-function 'BaseGroup-val :lambda-list '(m))
(cl:defmethod BaseGroup-val ((m <IKPlannerService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:BaseGroup-val is deprecated.  Use xr1controllerol-srv:BaseGroup instead.")
  (BaseGroup m))

(cl:ensure-generic-function 'Period-val :lambda-list '(m))
(cl:defmethod Period-val ((m <IKPlannerService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:Period-val is deprecated.  Use xr1controllerol-srv:Period instead.")
  (Period m))

(cl:ensure-generic-function 'PlannerMethod-val :lambda-list '(m))
(cl:defmethod PlannerMethod-val ((m <IKPlannerService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:PlannerMethod-val is deprecated.  Use xr1controllerol-srv:PlannerMethod instead.")
  (PlannerMethod m))

(cl:ensure-generic-function 'TargetTransform-val :lambda-list '(m))
(cl:defmethod TargetTransform-val ((m <IKPlannerService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:TargetTransform-val is deprecated.  Use xr1controllerol-srv:TargetTransform instead.")
  (TargetTransform m))

(cl:ensure-generic-function 'Reference1-val :lambda-list '(m))
(cl:defmethod Reference1-val ((m <IKPlannerService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:Reference1-val is deprecated.  Use xr1controllerol-srv:Reference1 instead.")
  (Reference1 m))

(cl:ensure-generic-function 'Reference2-val :lambda-list '(m))
(cl:defmethod Reference2-val ((m <IKPlannerService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:Reference2-val is deprecated.  Use xr1controllerol-srv:Reference2 instead.")
  (Reference2 m))

(cl:ensure-generic-function 'TargetElbowAngle-val :lambda-list '(m))
(cl:defmethod TargetElbowAngle-val ((m <IKPlannerService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:TargetElbowAngle-val is deprecated.  Use xr1controllerol-srv:TargetElbowAngle instead.")
  (TargetElbowAngle m))

(cl:ensure-generic-function 'Grip-val :lambda-list '(m))
(cl:defmethod Grip-val ((m <IKPlannerService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:Grip-val is deprecated.  Use xr1controllerol-srv:Grip instead.")
  (Grip m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <IKPlannerService-request>) ostream)
  "Serializes a message object of type '<IKPlannerService-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'NewTarget) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'ControlGroup)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'BaseGroup)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'Period))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'PlannerMethod)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'TargetTransform) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Reference1) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Reference2) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'TargetElbowAngle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'Grip) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <IKPlannerService-request>) istream)
  "Deserializes a message object of type '<IKPlannerService-request>"
    (cl:setf (cl:slot-value msg 'NewTarget) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ControlGroup) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'BaseGroup) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Period) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'PlannerMethod) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'TargetTransform) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Reference1) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Reference2) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'TargetElbowAngle) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'Grip) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<IKPlannerService-request>)))
  "Returns string type for a service object of type '<IKPlannerService-request>"
  "xr1controllerol/IKPlannerServiceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IKPlannerService-request)))
  "Returns string type for a service object of type 'IKPlannerService-request"
  "xr1controllerol/IKPlannerServiceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<IKPlannerService-request>)))
  "Returns md5sum for a message object of type '<IKPlannerService-request>"
  "349437e0afbeedef0081b8bf7a38d1fa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'IKPlannerService-request)))
  "Returns md5sum for a message object of type 'IKPlannerService-request"
  "349437e0afbeedef0081b8bf7a38d1fa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<IKPlannerService-request>)))
  "Returns full string definition for message of type '<IKPlannerService-request>"
  (cl:format cl:nil "bool NewTarget~%int32 ControlGroup~%int32 BaseGroup~%float64 Period~%int32 PlannerMethod~%geometry_msgs/Transform TargetTransform~%geometry_msgs/Vector3 Reference1~%geometry_msgs/Vector3 Reference2~%float64 TargetElbowAngle~%bool Grip~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'IKPlannerService-request)))
  "Returns full string definition for message of type 'IKPlannerService-request"
  (cl:format cl:nil "bool NewTarget~%int32 ControlGroup~%int32 BaseGroup~%float64 Period~%int32 PlannerMethod~%geometry_msgs/Transform TargetTransform~%geometry_msgs/Vector3 Reference1~%geometry_msgs/Vector3 Reference2~%float64 TargetElbowAngle~%bool Grip~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <IKPlannerService-request>))
  (cl:+ 0
     1
     4
     4
     8
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'TargetTransform))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Reference1))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Reference2))
     8
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <IKPlannerService-request>))
  "Converts a ROS message object to a list"
  (cl:list 'IKPlannerService-request
    (cl:cons ':NewTarget (NewTarget msg))
    (cl:cons ':ControlGroup (ControlGroup msg))
    (cl:cons ':BaseGroup (BaseGroup msg))
    (cl:cons ':Period (Period msg))
    (cl:cons ':PlannerMethod (PlannerMethod msg))
    (cl:cons ':TargetTransform (TargetTransform msg))
    (cl:cons ':Reference1 (Reference1 msg))
    (cl:cons ':Reference2 (Reference2 msg))
    (cl:cons ':TargetElbowAngle (TargetElbowAngle msg))
    (cl:cons ':Grip (Grip msg))
))
;//! \htmlinclude IKPlannerService-response.msg.html

(cl:defclass <IKPlannerService-response> (roslisp-msg-protocol:ros-message)
  ((inProgress
    :reader inProgress
    :initarg :inProgress
    :type cl:boolean
    :initform cl:nil)
   (isReachable
    :reader isReachable
    :initarg :isReachable
    :type cl:boolean
    :initform cl:nil)
   (isAccepted
    :reader isAccepted
    :initarg :isAccepted
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass IKPlannerService-response (<IKPlannerService-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <IKPlannerService-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'IKPlannerService-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xr1controllerol-srv:<IKPlannerService-response> is deprecated: use xr1controllerol-srv:IKPlannerService-response instead.")))

(cl:ensure-generic-function 'inProgress-val :lambda-list '(m))
(cl:defmethod inProgress-val ((m <IKPlannerService-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:inProgress-val is deprecated.  Use xr1controllerol-srv:inProgress instead.")
  (inProgress m))

(cl:ensure-generic-function 'isReachable-val :lambda-list '(m))
(cl:defmethod isReachable-val ((m <IKPlannerService-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:isReachable-val is deprecated.  Use xr1controllerol-srv:isReachable instead.")
  (isReachable m))

(cl:ensure-generic-function 'isAccepted-val :lambda-list '(m))
(cl:defmethod isAccepted-val ((m <IKPlannerService-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xr1controllerol-srv:isAccepted-val is deprecated.  Use xr1controllerol-srv:isAccepted instead.")
  (isAccepted m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <IKPlannerService-response>) ostream)
  "Serializes a message object of type '<IKPlannerService-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'inProgress) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'isReachable) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'isAccepted) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <IKPlannerService-response>) istream)
  "Deserializes a message object of type '<IKPlannerService-response>"
    (cl:setf (cl:slot-value msg 'inProgress) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'isReachable) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'isAccepted) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<IKPlannerService-response>)))
  "Returns string type for a service object of type '<IKPlannerService-response>"
  "xr1controllerol/IKPlannerServiceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IKPlannerService-response)))
  "Returns string type for a service object of type 'IKPlannerService-response"
  "xr1controllerol/IKPlannerServiceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<IKPlannerService-response>)))
  "Returns md5sum for a message object of type '<IKPlannerService-response>"
  "349437e0afbeedef0081b8bf7a38d1fa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'IKPlannerService-response)))
  "Returns md5sum for a message object of type 'IKPlannerService-response"
  "349437e0afbeedef0081b8bf7a38d1fa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<IKPlannerService-response>)))
  "Returns full string definition for message of type '<IKPlannerService-response>"
  (cl:format cl:nil "bool inProgress~%bool isReachable~%bool isAccepted~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'IKPlannerService-response)))
  "Returns full string definition for message of type 'IKPlannerService-response"
  (cl:format cl:nil "bool inProgress~%bool isReachable~%bool isAccepted~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <IKPlannerService-response>))
  (cl:+ 0
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <IKPlannerService-response>))
  "Converts a ROS message object to a list"
  (cl:list 'IKPlannerService-response
    (cl:cons ':inProgress (inProgress msg))
    (cl:cons ':isReachable (isReachable msg))
    (cl:cons ':isAccepted (isAccepted msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'IKPlannerService)))
  'IKPlannerService-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'IKPlannerService)))
  'IKPlannerService-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IKPlannerService)))
  "Returns string type for a service object of type '<IKPlannerService>"
  "xr1controllerol/IKPlannerService")