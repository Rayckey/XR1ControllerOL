;; Auto-generated. Do not edit!


(when (boundp 'xr1controllerol::BalanceQuery)
  (if (not (find-package "XR1CONTROLLEROL"))
    (make-package "XR1CONTROLLEROL"))
  (shadow 'BalanceQuery (find-package "XR1CONTROLLEROL")))
(unless (find-package "XR1CONTROLLEROL::BALANCEQUERY")
  (make-package "XR1CONTROLLEROL::BALANCEQUERY"))
(unless (find-package "XR1CONTROLLEROL::BALANCEQUERYREQUEST")
  (make-package "XR1CONTROLLEROL::BALANCEQUERYREQUEST"))
(unless (find-package "XR1CONTROLLEROL::BALANCEQUERYRESPONSE")
  (make-package "XR1CONTROLLEROL::BALANCEQUERYRESPONSE"))

(in-package "ROS")





(defclass xr1controllerol::BalanceQueryRequest
  :super ros::object
  :slots (_isQuery ))

(defmethod xr1controllerol::BalanceQueryRequest
  (:init
   (&key
    ((:isQuery __isQuery) nil)
    )
   (send-super :init)
   (setq _isQuery __isQuery)
   self)
  (:isQuery
   (&optional __isQuery)
   (if __isQuery (setq _isQuery __isQuery)) _isQuery)
  (:serialization-length
   ()
   (+
    ;; bool _isQuery
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _isQuery
       (if _isQuery (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _isQuery
     (setq _isQuery (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass xr1controllerol::BalanceQueryResponse
  :super ros::object
  :slots (_inBLCMode _hasIdle _hasPassive _hasActive ))

(defmethod xr1controllerol::BalanceQueryResponse
  (:init
   (&key
    ((:inBLCMode __inBLCMode) nil)
    ((:hasIdle __hasIdle) nil)
    ((:hasPassive __hasPassive) nil)
    ((:hasActive __hasActive) nil)
    )
   (send-super :init)
   (setq _inBLCMode __inBLCMode)
   (setq _hasIdle __hasIdle)
   (setq _hasPassive __hasPassive)
   (setq _hasActive __hasActive)
   self)
  (:inBLCMode
   (&optional __inBLCMode)
   (if __inBLCMode (setq _inBLCMode __inBLCMode)) _inBLCMode)
  (:hasIdle
   (&optional __hasIdle)
   (if __hasIdle (setq _hasIdle __hasIdle)) _hasIdle)
  (:hasPassive
   (&optional __hasPassive)
   (if __hasPassive (setq _hasPassive __hasPassive)) _hasPassive)
  (:hasActive
   (&optional __hasActive)
   (if __hasActive (setq _hasActive __hasActive)) _hasActive)
  (:serialization-length
   ()
   (+
    ;; bool _inBLCMode
    1
    ;; bool _hasIdle
    1
    ;; bool _hasPassive
    1
    ;; bool _hasActive
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _inBLCMode
       (if _inBLCMode (write-byte -1 s) (write-byte 0 s))
     ;; bool _hasIdle
       (if _hasIdle (write-byte -1 s) (write-byte 0 s))
     ;; bool _hasPassive
       (if _hasPassive (write-byte -1 s) (write-byte 0 s))
     ;; bool _hasActive
       (if _hasActive (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _inBLCMode
     (setq _inBLCMode (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _hasIdle
     (setq _hasIdle (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _hasPassive
     (setq _hasPassive (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _hasActive
     (setq _hasActive (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass xr1controllerol::BalanceQuery
  :super ros::object
  :slots ())

(setf (get xr1controllerol::BalanceQuery :md5sum-) "cd5d7de45f0cd959837e8340a01d26a2")
(setf (get xr1controllerol::BalanceQuery :datatype-) "xr1controllerol/BalanceQuery")
(setf (get xr1controllerol::BalanceQuery :request) xr1controllerol::BalanceQueryRequest)
(setf (get xr1controllerol::BalanceQuery :response) xr1controllerol::BalanceQueryResponse)

(defmethod xr1controllerol::BalanceQueryRequest
  (:response () (instance xr1controllerol::BalanceQueryResponse :init)))

(setf (get xr1controllerol::BalanceQueryRequest :md5sum-) "cd5d7de45f0cd959837e8340a01d26a2")
(setf (get xr1controllerol::BalanceQueryRequest :datatype-) "xr1controllerol/BalanceQueryRequest")
(setf (get xr1controllerol::BalanceQueryRequest :definition-)
      "bool isQuery
---
bool inBLCMode
bool hasIdle
bool hasPassive
bool hasActive


")

(setf (get xr1controllerol::BalanceQueryResponse :md5sum-) "cd5d7de45f0cd959837e8340a01d26a2")
(setf (get xr1controllerol::BalanceQueryResponse :datatype-) "xr1controllerol/BalanceQueryResponse")
(setf (get xr1controllerol::BalanceQueryResponse :definition-)
      "bool isQuery
---
bool inBLCMode
bool hasIdle
bool hasPassive
bool hasActive


")



(provide :xr1controllerol/BalanceQuery "cd5d7de45f0cd959837e8340a01d26a2")


