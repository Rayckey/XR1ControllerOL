;; Auto-generated. Do not edit!


(when (boundp 'xr1controllerol::AnimationMsgs)
  (if (not (find-package "XR1CONTROLLEROL"))
    (make-package "XR1CONTROLLEROL"))
  (shadow 'AnimationMsgs (find-package "XR1CONTROLLEROL")))
(unless (find-package "XR1CONTROLLEROL::ANIMATIONMSGS")
  (make-package "XR1CONTROLLEROL::ANIMATIONMSGS"))

(in-package "ROS")
;;//! \htmlinclude AnimationMsgs.msg.html


(defclass xr1controllerol::AnimationMsgs
  :super ros::object
  :slots (_AnimationType _AnimationID ))

(defmethod xr1controllerol::AnimationMsgs
  (:init
   (&key
    ((:AnimationType __AnimationType) 0)
    ((:AnimationID __AnimationID) 0)
    )
   (send-super :init)
   (setq _AnimationType (round __AnimationType))
   (setq _AnimationID (round __AnimationID))
   self)
  (:AnimationType
   (&optional __AnimationType)
   (if __AnimationType (setq _AnimationType __AnimationType)) _AnimationType)
  (:AnimationID
   (&optional __AnimationID)
   (if __AnimationID (setq _AnimationID __AnimationID)) _AnimationID)
  (:serialization-length
   ()
   (+
    ;; int32 _AnimationType
    4
    ;; int32 _AnimationID
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _AnimationType
       (write-long _AnimationType s)
     ;; int32 _AnimationID
       (write-long _AnimationID s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _AnimationType
     (setq _AnimationType (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _AnimationID
     (setq _AnimationID (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get xr1controllerol::AnimationMsgs :md5sum-) "9e4a55e887ede9012b0ab385278da204")
(setf (get xr1controllerol::AnimationMsgs :datatype-) "xr1controllerol/AnimationMsgs")
(setf (get xr1controllerol::AnimationMsgs :definition-)
      "int32 AnimationType
int32 AnimationID
")



(provide :xr1controllerol/AnimationMsgs "9e4a55e887ede9012b0ab385278da204")


