;; Auto-generated. Do not edit!


(when (boundp 'xr1controllerol::IKPlannerService)
  (if (not (find-package "XR1CONTROLLEROL"))
    (make-package "XR1CONTROLLEROL"))
  (shadow 'IKPlannerService (find-package "XR1CONTROLLEROL")))
(unless (find-package "XR1CONTROLLEROL::IKPLANNERSERVICE")
  (make-package "XR1CONTROLLEROL::IKPLANNERSERVICE"))
(unless (find-package "XR1CONTROLLEROL::IKPLANNERSERVICEREQUEST")
  (make-package "XR1CONTROLLEROL::IKPLANNERSERVICEREQUEST"))
(unless (find-package "XR1CONTROLLEROL::IKPLANNERSERVICERESPONSE")
  (make-package "XR1CONTROLLEROL::IKPLANNERSERVICERESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass xr1controllerol::IKPlannerServiceRequest
  :super ros::object
  :slots (_NewTarget _ControlGroup _BaseGroup _Period _PlannerMethod _TargetTransform _Reference1 _Reference2 _TargetElbowAngle _Grip ))

(defmethod xr1controllerol::IKPlannerServiceRequest
  (:init
   (&key
    ((:NewTarget __NewTarget) nil)
    ((:ControlGroup __ControlGroup) 0)
    ((:BaseGroup __BaseGroup) 0)
    ((:Period __Period) 0.0)
    ((:PlannerMethod __PlannerMethod) 0)
    ((:TargetTransform __TargetTransform) (instance geometry_msgs::Transform :init))
    ((:Reference1 __Reference1) (instance geometry_msgs::Vector3 :init))
    ((:Reference2 __Reference2) (instance geometry_msgs::Vector3 :init))
    ((:TargetElbowAngle __TargetElbowAngle) 0.0)
    ((:Grip __Grip) nil)
    )
   (send-super :init)
   (setq _NewTarget __NewTarget)
   (setq _ControlGroup (round __ControlGroup))
   (setq _BaseGroup (round __BaseGroup))
   (setq _Period (float __Period))
   (setq _PlannerMethod (round __PlannerMethod))
   (setq _TargetTransform __TargetTransform)
   (setq _Reference1 __Reference1)
   (setq _Reference2 __Reference2)
   (setq _TargetElbowAngle (float __TargetElbowAngle))
   (setq _Grip __Grip)
   self)
  (:NewTarget
   (&optional __NewTarget)
   (if __NewTarget (setq _NewTarget __NewTarget)) _NewTarget)
  (:ControlGroup
   (&optional __ControlGroup)
   (if __ControlGroup (setq _ControlGroup __ControlGroup)) _ControlGroup)
  (:BaseGroup
   (&optional __BaseGroup)
   (if __BaseGroup (setq _BaseGroup __BaseGroup)) _BaseGroup)
  (:Period
   (&optional __Period)
   (if __Period (setq _Period __Period)) _Period)
  (:PlannerMethod
   (&optional __PlannerMethod)
   (if __PlannerMethod (setq _PlannerMethod __PlannerMethod)) _PlannerMethod)
  (:TargetTransform
   (&rest __TargetTransform)
   (if (keywordp (car __TargetTransform))
       (send* _TargetTransform __TargetTransform)
     (progn
       (if __TargetTransform (setq _TargetTransform (car __TargetTransform)))
       _TargetTransform)))
  (:Reference1
   (&rest __Reference1)
   (if (keywordp (car __Reference1))
       (send* _Reference1 __Reference1)
     (progn
       (if __Reference1 (setq _Reference1 (car __Reference1)))
       _Reference1)))
  (:Reference2
   (&rest __Reference2)
   (if (keywordp (car __Reference2))
       (send* _Reference2 __Reference2)
     (progn
       (if __Reference2 (setq _Reference2 (car __Reference2)))
       _Reference2)))
  (:TargetElbowAngle
   (&optional __TargetElbowAngle)
   (if __TargetElbowAngle (setq _TargetElbowAngle __TargetElbowAngle)) _TargetElbowAngle)
  (:Grip
   (&optional __Grip)
   (if __Grip (setq _Grip __Grip)) _Grip)
  (:serialization-length
   ()
   (+
    ;; bool _NewTarget
    1
    ;; int32 _ControlGroup
    4
    ;; int32 _BaseGroup
    4
    ;; float64 _Period
    8
    ;; int32 _PlannerMethod
    4
    ;; geometry_msgs/Transform _TargetTransform
    (send _TargetTransform :serialization-length)
    ;; geometry_msgs/Vector3 _Reference1
    (send _Reference1 :serialization-length)
    ;; geometry_msgs/Vector3 _Reference2
    (send _Reference2 :serialization-length)
    ;; float64 _TargetElbowAngle
    8
    ;; bool _Grip
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _NewTarget
       (if _NewTarget (write-byte -1 s) (write-byte 0 s))
     ;; int32 _ControlGroup
       (write-long _ControlGroup s)
     ;; int32 _BaseGroup
       (write-long _BaseGroup s)
     ;; float64 _Period
       (sys::poke _Period (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int32 _PlannerMethod
       (write-long _PlannerMethod s)
     ;; geometry_msgs/Transform _TargetTransform
       (send _TargetTransform :serialize s)
     ;; geometry_msgs/Vector3 _Reference1
       (send _Reference1 :serialize s)
     ;; geometry_msgs/Vector3 _Reference2
       (send _Reference2 :serialize s)
     ;; float64 _TargetElbowAngle
       (sys::poke _TargetElbowAngle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _Grip
       (if _Grip (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _NewTarget
     (setq _NewTarget (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _ControlGroup
     (setq _ControlGroup (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _BaseGroup
     (setq _BaseGroup (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _Period
     (setq _Period (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int32 _PlannerMethod
     (setq _PlannerMethod (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/Transform _TargetTransform
     (send _TargetTransform :deserialize buf ptr-) (incf ptr- (send _TargetTransform :serialization-length))
   ;; geometry_msgs/Vector3 _Reference1
     (send _Reference1 :deserialize buf ptr-) (incf ptr- (send _Reference1 :serialization-length))
   ;; geometry_msgs/Vector3 _Reference2
     (send _Reference2 :deserialize buf ptr-) (incf ptr- (send _Reference2 :serialization-length))
   ;; float64 _TargetElbowAngle
     (setq _TargetElbowAngle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _Grip
     (setq _Grip (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass xr1controllerol::IKPlannerServiceResponse
  :super ros::object
  :slots (_inProgress _isReachable _isAccepted ))

(defmethod xr1controllerol::IKPlannerServiceResponse
  (:init
   (&key
    ((:inProgress __inProgress) nil)
    ((:isReachable __isReachable) nil)
    ((:isAccepted __isAccepted) nil)
    )
   (send-super :init)
   (setq _inProgress __inProgress)
   (setq _isReachable __isReachable)
   (setq _isAccepted __isAccepted)
   self)
  (:inProgress
   (&optional __inProgress)
   (if __inProgress (setq _inProgress __inProgress)) _inProgress)
  (:isReachable
   (&optional __isReachable)
   (if __isReachable (setq _isReachable __isReachable)) _isReachable)
  (:isAccepted
   (&optional __isAccepted)
   (if __isAccepted (setq _isAccepted __isAccepted)) _isAccepted)
  (:serialization-length
   ()
   (+
    ;; bool _inProgress
    1
    ;; bool _isReachable
    1
    ;; bool _isAccepted
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _inProgress
       (if _inProgress (write-byte -1 s) (write-byte 0 s))
     ;; bool _isReachable
       (if _isReachable (write-byte -1 s) (write-byte 0 s))
     ;; bool _isAccepted
       (if _isAccepted (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _inProgress
     (setq _inProgress (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _isReachable
     (setq _isReachable (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _isAccepted
     (setq _isAccepted (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass xr1controllerol::IKPlannerService
  :super ros::object
  :slots ())

(setf (get xr1controllerol::IKPlannerService :md5sum-) "349437e0afbeedef0081b8bf7a38d1fa")
(setf (get xr1controllerol::IKPlannerService :datatype-) "xr1controllerol/IKPlannerService")
(setf (get xr1controllerol::IKPlannerService :request) xr1controllerol::IKPlannerServiceRequest)
(setf (get xr1controllerol::IKPlannerService :response) xr1controllerol::IKPlannerServiceResponse)

(defmethod xr1controllerol::IKPlannerServiceRequest
  (:response () (instance xr1controllerol::IKPlannerServiceResponse :init)))

(setf (get xr1controllerol::IKPlannerServiceRequest :md5sum-) "349437e0afbeedef0081b8bf7a38d1fa")
(setf (get xr1controllerol::IKPlannerServiceRequest :datatype-) "xr1controllerol/IKPlannerServiceRequest")
(setf (get xr1controllerol::IKPlannerServiceRequest :definition-)
      "bool NewTarget
int32 ControlGroup
int32 BaseGroup
float64 Period
int32 PlannerMethod
geometry_msgs/Transform TargetTransform
geometry_msgs/Vector3 Reference1
geometry_msgs/Vector3 Reference2
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
---
bool inProgress
bool isReachable
bool isAccepted

")

(setf (get xr1controllerol::IKPlannerServiceResponse :md5sum-) "349437e0afbeedef0081b8bf7a38d1fa")
(setf (get xr1controllerol::IKPlannerServiceResponse :datatype-) "xr1controllerol/IKPlannerServiceResponse")
(setf (get xr1controllerol::IKPlannerServiceResponse :definition-)
      "bool NewTarget
int32 ControlGroup
int32 BaseGroup
float64 Period
int32 PlannerMethod
geometry_msgs/Transform TargetTransform
geometry_msgs/Vector3 Reference1
geometry_msgs/Vector3 Reference2
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
---
bool inProgress
bool isReachable
bool isAccepted

")



(provide :xr1controllerol/IKPlannerService "349437e0afbeedef0081b8bf7a38d1fa")


