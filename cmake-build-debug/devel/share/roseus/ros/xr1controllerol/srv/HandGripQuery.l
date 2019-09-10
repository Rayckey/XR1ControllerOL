;; Auto-generated. Do not edit!


(when (boundp 'xr1controllerol::HandGripQuery)
  (if (not (find-package "XR1CONTROLLEROL"))
    (make-package "XR1CONTROLLEROL"))
  (shadow 'HandGripQuery (find-package "XR1CONTROLLEROL")))
(unless (find-package "XR1CONTROLLEROL::HANDGRIPQUERY")
  (make-package "XR1CONTROLLEROL::HANDGRIPQUERY"))
(unless (find-package "XR1CONTROLLEROL::HANDGRIPQUERYREQUEST")
  (make-package "XR1CONTROLLEROL::HANDGRIPQUERYREQUEST"))
(unless (find-package "XR1CONTROLLEROL::HANDGRIPQUERYRESPONSE")
  (make-package "XR1CONTROLLEROL::HANDGRIPQUERYRESPONSE"))

(in-package "ROS")





(defclass xr1controllerol::HandGripQueryRequest
  :super ros::object
  :slots (_ControlGroup ))

(defmethod xr1controllerol::HandGripQueryRequest
  (:init
   (&key
    ((:ControlGroup __ControlGroup) 0)
    )
   (send-super :init)
   (setq _ControlGroup (round __ControlGroup))
   self)
  (:ControlGroup
   (&optional __ControlGroup)
   (if __ControlGroup (setq _ControlGroup __ControlGroup)) _ControlGroup)
  (:serialization-length
   ()
   (+
    ;; int32 _ControlGroup
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _ControlGroup
       (write-long _ControlGroup s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _ControlGroup
     (setq _ControlGroup (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass xr1controllerol::HandGripQueryResponse
  :super ros::object
  :slots (_inProgress _isGripped ))

(defmethod xr1controllerol::HandGripQueryResponse
  (:init
   (&key
    ((:inProgress __inProgress) nil)
    ((:isGripped __isGripped) nil)
    )
   (send-super :init)
   (setq _inProgress __inProgress)
   (setq _isGripped __isGripped)
   self)
  (:inProgress
   (&optional __inProgress)
   (if __inProgress (setq _inProgress __inProgress)) _inProgress)
  (:isGripped
   (&optional __isGripped)
   (if __isGripped (setq _isGripped __isGripped)) _isGripped)
  (:serialization-length
   ()
   (+
    ;; bool _inProgress
    1
    ;; bool _isGripped
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _inProgress
       (if _inProgress (write-byte -1 s) (write-byte 0 s))
     ;; bool _isGripped
       (if _isGripped (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _inProgress
     (setq _inProgress (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _isGripped
     (setq _isGripped (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass xr1controllerol::HandGripQuery
  :super ros::object
  :slots ())

(setf (get xr1controllerol::HandGripQuery :md5sum-) "6779f448455fbe1c7c9fa9fca9457608")
(setf (get xr1controllerol::HandGripQuery :datatype-) "xr1controllerol/HandGripQuery")
(setf (get xr1controllerol::HandGripQuery :request) xr1controllerol::HandGripQueryRequest)
(setf (get xr1controllerol::HandGripQuery :response) xr1controllerol::HandGripQueryResponse)

(defmethod xr1controllerol::HandGripQueryRequest
  (:response () (instance xr1controllerol::HandGripQueryResponse :init)))

(setf (get xr1controllerol::HandGripQueryRequest :md5sum-) "6779f448455fbe1c7c9fa9fca9457608")
(setf (get xr1controllerol::HandGripQueryRequest :datatype-) "xr1controllerol/HandGripQueryRequest")
(setf (get xr1controllerol::HandGripQueryRequest :definition-)
      "int32 ControlGroup
---
bool inProgress
bool isGripped


")

(setf (get xr1controllerol::HandGripQueryResponse :md5sum-) "6779f448455fbe1c7c9fa9fca9457608")
(setf (get xr1controllerol::HandGripQueryResponse :datatype-) "xr1controllerol/HandGripQueryResponse")
(setf (get xr1controllerol::HandGripQueryResponse :definition-)
      "int32 ControlGroup
---
bool inProgress
bool isGripped


")



(provide :xr1controllerol/HandGripQuery "6779f448455fbe1c7c9fa9fca9457608")


