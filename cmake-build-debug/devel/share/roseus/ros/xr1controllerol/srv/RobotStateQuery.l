;; Auto-generated. Do not edit!


(when (boundp 'xr1controllerol::RobotStateQuery)
  (if (not (find-package "XR1CONTROLLEROL"))
    (make-package "XR1CONTROLLEROL"))
  (shadow 'RobotStateQuery (find-package "XR1CONTROLLEROL")))
(unless (find-package "XR1CONTROLLEROL::ROBOTSTATEQUERY")
  (make-package "XR1CONTROLLEROL::ROBOTSTATEQUERY"))
(unless (find-package "XR1CONTROLLEROL::ROBOTSTATEQUERYREQUEST")
  (make-package "XR1CONTROLLEROL::ROBOTSTATEQUERYREQUEST"))
(unless (find-package "XR1CONTROLLEROL::ROBOTSTATEQUERYRESPONSE")
  (make-package "XR1CONTROLLEROL::ROBOTSTATEQUERYRESPONSE"))

(in-package "ROS")





(defclass xr1controllerol::RobotStateQueryRequest
  :super ros::object
  :slots (_isQuery _requestLift ))

(defmethod xr1controllerol::RobotStateQueryRequest
  (:init
   (&key
    ((:isQuery __isQuery) nil)
    ((:requestLift __requestLift) nil)
    )
   (send-super :init)
   (setq _isQuery __isQuery)
   (setq _requestLift __requestLift)
   self)
  (:isQuery
   (&optional __isQuery)
   (if __isQuery (setq _isQuery __isQuery)) _isQuery)
  (:requestLift
   (&optional __requestLift)
   (if __requestLift (setq _requestLift __requestLift)) _requestLift)
  (:serialization-length
   ()
   (+
    ;; bool _isQuery
    1
    ;; bool _requestLift
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _isQuery
       (if _isQuery (write-byte -1 s) (write-byte 0 s))
     ;; bool _requestLift
       (if _requestLift (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _isQuery
     (setq _isQuery (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _requestLift
     (setq _requestLift (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass xr1controllerol::RobotStateQueryResponse
  :super ros::object
  :slots (_isOkay _RobotState _CollisionSwitch _MainBodyMode _HeadBodyMode _LeftArmMode _RightArmMode _LeftHandMode _RightHandMode ))

(defmethod xr1controllerol::RobotStateQueryResponse
  (:init
   (&key
    ((:isOkay __isOkay) nil)
    ((:RobotState __RobotState) 0)
    ((:CollisionSwitch __CollisionSwitch) nil)
    ((:MainBodyMode __MainBodyMode) 0)
    ((:HeadBodyMode __HeadBodyMode) 0)
    ((:LeftArmMode __LeftArmMode) 0)
    ((:RightArmMode __RightArmMode) 0)
    ((:LeftHandMode __LeftHandMode) 0)
    ((:RightHandMode __RightHandMode) 0)
    )
   (send-super :init)
   (setq _isOkay __isOkay)
   (setq _RobotState (round __RobotState))
   (setq _CollisionSwitch __CollisionSwitch)
   (setq _MainBodyMode (round __MainBodyMode))
   (setq _HeadBodyMode (round __HeadBodyMode))
   (setq _LeftArmMode (round __LeftArmMode))
   (setq _RightArmMode (round __RightArmMode))
   (setq _LeftHandMode (round __LeftHandMode))
   (setq _RightHandMode (round __RightHandMode))
   self)
  (:isOkay
   (&optional __isOkay)
   (if __isOkay (setq _isOkay __isOkay)) _isOkay)
  (:RobotState
   (&optional __RobotState)
   (if __RobotState (setq _RobotState __RobotState)) _RobotState)
  (:CollisionSwitch
   (&optional __CollisionSwitch)
   (if __CollisionSwitch (setq _CollisionSwitch __CollisionSwitch)) _CollisionSwitch)
  (:MainBodyMode
   (&optional __MainBodyMode)
   (if __MainBodyMode (setq _MainBodyMode __MainBodyMode)) _MainBodyMode)
  (:HeadBodyMode
   (&optional __HeadBodyMode)
   (if __HeadBodyMode (setq _HeadBodyMode __HeadBodyMode)) _HeadBodyMode)
  (:LeftArmMode
   (&optional __LeftArmMode)
   (if __LeftArmMode (setq _LeftArmMode __LeftArmMode)) _LeftArmMode)
  (:RightArmMode
   (&optional __RightArmMode)
   (if __RightArmMode (setq _RightArmMode __RightArmMode)) _RightArmMode)
  (:LeftHandMode
   (&optional __LeftHandMode)
   (if __LeftHandMode (setq _LeftHandMode __LeftHandMode)) _LeftHandMode)
  (:RightHandMode
   (&optional __RightHandMode)
   (if __RightHandMode (setq _RightHandMode __RightHandMode)) _RightHandMode)
  (:serialization-length
   ()
   (+
    ;; bool _isOkay
    1
    ;; int16 _RobotState
    2
    ;; bool _CollisionSwitch
    1
    ;; int16 _MainBodyMode
    2
    ;; int16 _HeadBodyMode
    2
    ;; int16 _LeftArmMode
    2
    ;; int16 _RightArmMode
    2
    ;; int16 _LeftHandMode
    2
    ;; int16 _RightHandMode
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _isOkay
       (if _isOkay (write-byte -1 s) (write-byte 0 s))
     ;; int16 _RobotState
       (write-word _RobotState s)
     ;; bool _CollisionSwitch
       (if _CollisionSwitch (write-byte -1 s) (write-byte 0 s))
     ;; int16 _MainBodyMode
       (write-word _MainBodyMode s)
     ;; int16 _HeadBodyMode
       (write-word _HeadBodyMode s)
     ;; int16 _LeftArmMode
       (write-word _LeftArmMode s)
     ;; int16 _RightArmMode
       (write-word _RightArmMode s)
     ;; int16 _LeftHandMode
       (write-word _LeftHandMode s)
     ;; int16 _RightHandMode
       (write-word _RightHandMode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _isOkay
     (setq _isOkay (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int16 _RobotState
     (setq _RobotState (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; bool _CollisionSwitch
     (setq _CollisionSwitch (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int16 _MainBodyMode
     (setq _MainBodyMode (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _HeadBodyMode
     (setq _HeadBodyMode (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _LeftArmMode
     (setq _LeftArmMode (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _RightArmMode
     (setq _RightArmMode (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _LeftHandMode
     (setq _LeftHandMode (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _RightHandMode
     (setq _RightHandMode (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(defclass xr1controllerol::RobotStateQuery
  :super ros::object
  :slots ())

(setf (get xr1controllerol::RobotStateQuery :md5sum-) "7e1984b1a9b270399515e3ff3569d4d2")
(setf (get xr1controllerol::RobotStateQuery :datatype-) "xr1controllerol/RobotStateQuery")
(setf (get xr1controllerol::RobotStateQuery :request) xr1controllerol::RobotStateQueryRequest)
(setf (get xr1controllerol::RobotStateQuery :response) xr1controllerol::RobotStateQueryResponse)

(defmethod xr1controllerol::RobotStateQueryRequest
  (:response () (instance xr1controllerol::RobotStateQueryResponse :init)))

(setf (get xr1controllerol::RobotStateQueryRequest :md5sum-) "7e1984b1a9b270399515e3ff3569d4d2")
(setf (get xr1controllerol::RobotStateQueryRequest :datatype-) "xr1controllerol/RobotStateQueryRequest")
(setf (get xr1controllerol::RobotStateQueryRequest :definition-)
      "bool isQuery
bool requestLift
---
bool isOkay
int16 RobotState
bool CollisionSwitch
int16 MainBodyMode
int16 HeadBodyMode
int16 LeftArmMode
int16 RightArmMode
int16 LeftHandMode
int16 RightHandMode



")

(setf (get xr1controllerol::RobotStateQueryResponse :md5sum-) "7e1984b1a9b270399515e3ff3569d4d2")
(setf (get xr1controllerol::RobotStateQueryResponse :datatype-) "xr1controllerol/RobotStateQueryResponse")
(setf (get xr1controllerol::RobotStateQueryResponse :definition-)
      "bool isQuery
bool requestLift
---
bool isOkay
int16 RobotState
bool CollisionSwitch
int16 MainBodyMode
int16 HeadBodyMode
int16 LeftArmMode
int16 RightArmMode
int16 LeftHandMode
int16 RightHandMode



")



(provide :xr1controllerol/RobotStateQuery "7e1984b1a9b270399515e3ff3569d4d2")


