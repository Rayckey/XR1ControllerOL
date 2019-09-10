;; Auto-generated. Do not edit!


(when (boundp 'xr1controllerol::askReadiness)
  (if (not (find-package "XR1CONTROLLEROL"))
    (make-package "XR1CONTROLLEROL"))
  (shadow 'askReadiness (find-package "XR1CONTROLLEROL")))
(unless (find-package "XR1CONTROLLEROL::ASKREADINESS")
  (make-package "XR1CONTROLLEROL::ASKREADINESS"))
(unless (find-package "XR1CONTROLLEROL::ASKREADINESSREQUEST")
  (make-package "XR1CONTROLLEROL::ASKREADINESSREQUEST"))
(unless (find-package "XR1CONTROLLEROL::ASKREADINESSRESPONSE")
  (make-package "XR1CONTROLLEROL::ASKREADINESSRESPONSE"))

(in-package "ROS")





(defclass xr1controllerol::askReadinessRequest
  :super ros::object
  :slots (_isAsking ))

(defmethod xr1controllerol::askReadinessRequest
  (:init
   (&key
    ((:isAsking __isAsking) nil)
    )
   (send-super :init)
   (setq _isAsking __isAsking)
   self)
  (:isAsking
   (&optional __isAsking)
   (if __isAsking (setq _isAsking __isAsking)) _isAsking)
  (:serialization-length
   ()
   (+
    ;; bool _isAsking
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _isAsking
       (if _isAsking (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _isAsking
     (setq _isAsking (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass xr1controllerol::askReadinessResponse
  :super ros::object
  :slots (_isReady ))

(defmethod xr1controllerol::askReadinessResponse
  (:init
   (&key
    ((:isReady __isReady) nil)
    )
   (send-super :init)
   (setq _isReady __isReady)
   self)
  (:isReady
   (&optional __isReady)
   (if __isReady (setq _isReady __isReady)) _isReady)
  (:serialization-length
   ()
   (+
    ;; bool _isReady
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _isReady
       (if _isReady (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _isReady
     (setq _isReady (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass xr1controllerol::askReadiness
  :super ros::object
  :slots ())

(setf (get xr1controllerol::askReadiness :md5sum-) "4a4751acb64765569565cc0a23cfd7be")
(setf (get xr1controllerol::askReadiness :datatype-) "xr1controllerol/askReadiness")
(setf (get xr1controllerol::askReadiness :request) xr1controllerol::askReadinessRequest)
(setf (get xr1controllerol::askReadiness :response) xr1controllerol::askReadinessResponse)

(defmethod xr1controllerol::askReadinessRequest
  (:response () (instance xr1controllerol::askReadinessResponse :init)))

(setf (get xr1controllerol::askReadinessRequest :md5sum-) "4a4751acb64765569565cc0a23cfd7be")
(setf (get xr1controllerol::askReadinessRequest :datatype-) "xr1controllerol/askReadinessRequest")
(setf (get xr1controllerol::askReadinessRequest :definition-)
      "bool isAsking
---
bool isReady


")

(setf (get xr1controllerol::askReadinessResponse :md5sum-) "4a4751acb64765569565cc0a23cfd7be")
(setf (get xr1controllerol::askReadinessResponse :datatype-) "xr1controllerol/askReadinessResponse")
(setf (get xr1controllerol::askReadinessResponse :definition-)
      "bool isAsking
---
bool isReady


")



(provide :xr1controllerol/askReadiness "4a4751acb64765569565cc0a23cfd7be")


