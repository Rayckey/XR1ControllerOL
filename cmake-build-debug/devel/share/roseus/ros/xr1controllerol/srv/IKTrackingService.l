;; Auto-generated. Do not edit!


(when (boundp 'xr1controllerol::IKTrackingService)
  (if (not (find-package "XR1CONTROLLEROL"))
    (make-package "XR1CONTROLLEROL"))
  (shadow 'IKTrackingService (find-package "XR1CONTROLLEROL")))
(unless (find-package "XR1CONTROLLEROL::IKTRACKINGSERVICE")
  (make-package "XR1CONTROLLEROL::IKTRACKINGSERVICE"))
(unless (find-package "XR1CONTROLLEROL::IKTRACKINGSERVICEREQUEST")
  (make-package "XR1CONTROLLEROL::IKTRACKINGSERVICEREQUEST"))
(unless (find-package "XR1CONTROLLEROL::IKTRACKINGSERVICERESPONSE")
  (make-package "XR1CONTROLLEROL::IKTRACKINGSERVICERESPONSE"))

(in-package "ROS")





(defclass xr1controllerol::IKTrackingServiceRequest
  :super ros::object
  :slots (_NewTarget _ControlGroup ))

(defmethod xr1controllerol::IKTrackingServiceRequest
  (:init
   (&key
    ((:NewTarget __NewTarget) nil)
    ((:ControlGroup __ControlGroup) 0)
    )
   (send-super :init)
   (setq _NewTarget __NewTarget)
   (setq _ControlGroup (round __ControlGroup))
   self)
  (:NewTarget
   (&optional __NewTarget)
   (if __NewTarget (setq _NewTarget __NewTarget)) _NewTarget)
  (:ControlGroup
   (&optional __ControlGroup)
   (if __ControlGroup (setq _ControlGroup __ControlGroup)) _ControlGroup)
  (:serialization-length
   ()
   (+
    ;; bool _NewTarget
    1
    ;; int32 _ControlGroup
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _NewTarget
       (if _NewTarget (write-byte -1 s) (write-byte 0 s))
     ;; int32 _ControlGroup
       (write-long _ControlGroup s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _NewTarget
     (setq _NewTarget (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _ControlGroup
     (setq _ControlGroup (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass xr1controllerol::IKTrackingServiceResponse
  :super ros::object
  :slots (_isReachable _isAccepted _inProgress ))

(defmethod xr1controllerol::IKTrackingServiceResponse
  (:init
   (&key
    ((:isReachable __isReachable) nil)
    ((:isAccepted __isAccepted) nil)
    ((:inProgress __inProgress) nil)
    )
   (send-super :init)
   (setq _isReachable __isReachable)
   (setq _isAccepted __isAccepted)
   (setq _inProgress __inProgress)
   self)
  (:isReachable
   (&optional __isReachable)
   (if __isReachable (setq _isReachable __isReachable)) _isReachable)
  (:isAccepted
   (&optional __isAccepted)
   (if __isAccepted (setq _isAccepted __isAccepted)) _isAccepted)
  (:inProgress
   (&optional __inProgress)
   (if __inProgress (setq _inProgress __inProgress)) _inProgress)
  (:serialization-length
   ()
   (+
    ;; bool _isReachable
    1
    ;; bool _isAccepted
    1
    ;; bool _inProgress
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _isReachable
       (if _isReachable (write-byte -1 s) (write-byte 0 s))
     ;; bool _isAccepted
       (if _isAccepted (write-byte -1 s) (write-byte 0 s))
     ;; bool _inProgress
       (if _inProgress (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _isReachable
     (setq _isReachable (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _isAccepted
     (setq _isAccepted (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _inProgress
     (setq _inProgress (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass xr1controllerol::IKTrackingService
  :super ros::object
  :slots ())

(setf (get xr1controllerol::IKTrackingService :md5sum-) "c77f0c416d0a5b9b08f361d079852e35")
(setf (get xr1controllerol::IKTrackingService :datatype-) "xr1controllerol/IKTrackingService")
(setf (get xr1controllerol::IKTrackingService :request) xr1controllerol::IKTrackingServiceRequest)
(setf (get xr1controllerol::IKTrackingService :response) xr1controllerol::IKTrackingServiceResponse)

(defmethod xr1controllerol::IKTrackingServiceRequest
  (:response () (instance xr1controllerol::IKTrackingServiceResponse :init)))

(setf (get xr1controllerol::IKTrackingServiceRequest :md5sum-) "c77f0c416d0a5b9b08f361d079852e35")
(setf (get xr1controllerol::IKTrackingServiceRequest :datatype-) "xr1controllerol/IKTrackingServiceRequest")
(setf (get xr1controllerol::IKTrackingServiceRequest :definition-)
      "bool NewTarget
int32 ControlGroup
---
bool isReachable
bool isAccepted
bool inProgress
")

(setf (get xr1controllerol::IKTrackingServiceResponse :md5sum-) "c77f0c416d0a5b9b08f361d079852e35")
(setf (get xr1controllerol::IKTrackingServiceResponse :datatype-) "xr1controllerol/IKTrackingServiceResponse")
(setf (get xr1controllerol::IKTrackingServiceResponse :definition-)
      "bool NewTarget
int32 ControlGroup
---
bool isReachable
bool isAccepted
bool inProgress
")



(provide :xr1controllerol/IKTrackingService "c77f0c416d0a5b9b08f361d079852e35")


