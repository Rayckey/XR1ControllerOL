;; Auto-generated. Do not edit!


(when (boundp 'xr1controllerol::AnimationQuery)
  (if (not (find-package "XR1CONTROLLEROL"))
    (make-package "XR1CONTROLLEROL"))
  (shadow 'AnimationQuery (find-package "XR1CONTROLLEROL")))
(unless (find-package "XR1CONTROLLEROL::ANIMATIONQUERY")
  (make-package "XR1CONTROLLEROL::ANIMATIONQUERY"))
(unless (find-package "XR1CONTROLLEROL::ANIMATIONQUERYREQUEST")
  (make-package "XR1CONTROLLEROL::ANIMATIONQUERYREQUEST"))
(unless (find-package "XR1CONTROLLEROL::ANIMATIONQUERYRESPONSE")
  (make-package "XR1CONTROLLEROL::ANIMATIONQUERYRESPONSE"))

(in-package "ROS")





(defclass xr1controllerol::AnimationQueryRequest
  :super ros::object
  :slots (_isQuery ))

(defmethod xr1controllerol::AnimationQueryRequest
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

(defclass xr1controllerol::AnimationQueryResponse
  :super ros::object
  :slots (_inAnimationMode _isPlaying _hasDefault _hasIdle _AnimationType _AnimationID _AnimationProgress ))

(defmethod xr1controllerol::AnimationQueryResponse
  (:init
   (&key
    ((:inAnimationMode __inAnimationMode) nil)
    ((:isPlaying __isPlaying) nil)
    ((:hasDefault __hasDefault) nil)
    ((:hasIdle __hasIdle) nil)
    ((:AnimationType __AnimationType) 0)
    ((:AnimationID __AnimationID) 0)
    ((:AnimationProgress __AnimationProgress) 0)
    )
   (send-super :init)
   (setq _inAnimationMode __inAnimationMode)
   (setq _isPlaying __isPlaying)
   (setq _hasDefault __hasDefault)
   (setq _hasIdle __hasIdle)
   (setq _AnimationType (round __AnimationType))
   (setq _AnimationID (round __AnimationID))
   (setq _AnimationProgress (round __AnimationProgress))
   self)
  (:inAnimationMode
   (&optional __inAnimationMode)
   (if __inAnimationMode (setq _inAnimationMode __inAnimationMode)) _inAnimationMode)
  (:isPlaying
   (&optional __isPlaying)
   (if __isPlaying (setq _isPlaying __isPlaying)) _isPlaying)
  (:hasDefault
   (&optional __hasDefault)
   (if __hasDefault (setq _hasDefault __hasDefault)) _hasDefault)
  (:hasIdle
   (&optional __hasIdle)
   (if __hasIdle (setq _hasIdle __hasIdle)) _hasIdle)
  (:AnimationType
   (&optional __AnimationType)
   (if __AnimationType (setq _AnimationType __AnimationType)) _AnimationType)
  (:AnimationID
   (&optional __AnimationID)
   (if __AnimationID (setq _AnimationID __AnimationID)) _AnimationID)
  (:AnimationProgress
   (&optional __AnimationProgress)
   (if __AnimationProgress (setq _AnimationProgress __AnimationProgress)) _AnimationProgress)
  (:serialization-length
   ()
   (+
    ;; bool _inAnimationMode
    1
    ;; bool _isPlaying
    1
    ;; bool _hasDefault
    1
    ;; bool _hasIdle
    1
    ;; int64 _AnimationType
    8
    ;; int64 _AnimationID
    8
    ;; int64 _AnimationProgress
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _inAnimationMode
       (if _inAnimationMode (write-byte -1 s) (write-byte 0 s))
     ;; bool _isPlaying
       (if _isPlaying (write-byte -1 s) (write-byte 0 s))
     ;; bool _hasDefault
       (if _hasDefault (write-byte -1 s) (write-byte 0 s))
     ;; bool _hasIdle
       (if _hasIdle (write-byte -1 s) (write-byte 0 s))
     ;; int64 _AnimationType
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _AnimationType (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _AnimationType) (= (length (_AnimationType . bv)) 2)) ;; bignum
              (write-long (ash (elt (_AnimationType . bv) 0) 0) s)
              (write-long (ash (elt (_AnimationType . bv) 1) -1) s))
             ((and (class _AnimationType) (= (length (_AnimationType . bv)) 1)) ;; big1
              (write-long (elt (_AnimationType . bv) 0) s)
              (write-long (if (>= _AnimationType 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _AnimationType s)(write-long (if (>= _AnimationType 0) 0 #xffffffff) s)))
     ;; int64 _AnimationID
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _AnimationID (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _AnimationID) (= (length (_AnimationID . bv)) 2)) ;; bignum
              (write-long (ash (elt (_AnimationID . bv) 0) 0) s)
              (write-long (ash (elt (_AnimationID . bv) 1) -1) s))
             ((and (class _AnimationID) (= (length (_AnimationID . bv)) 1)) ;; big1
              (write-long (elt (_AnimationID . bv) 0) s)
              (write-long (if (>= _AnimationID 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _AnimationID s)(write-long (if (>= _AnimationID 0) 0 #xffffffff) s)))
     ;; int64 _AnimationProgress
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _AnimationProgress (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _AnimationProgress) (= (length (_AnimationProgress . bv)) 2)) ;; bignum
              (write-long (ash (elt (_AnimationProgress . bv) 0) 0) s)
              (write-long (ash (elt (_AnimationProgress . bv) 1) -1) s))
             ((and (class _AnimationProgress) (= (length (_AnimationProgress . bv)) 1)) ;; big1
              (write-long (elt (_AnimationProgress . bv) 0) s)
              (write-long (if (>= _AnimationProgress 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _AnimationProgress s)(write-long (if (>= _AnimationProgress 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _inAnimationMode
     (setq _inAnimationMode (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _isPlaying
     (setq _isPlaying (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _hasDefault
     (setq _hasDefault (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _hasIdle
     (setq _hasIdle (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int64 _AnimationType
#+(or :alpha :irix6 :x86_64)
      (setf _AnimationType (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _AnimationType (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _AnimationID
#+(or :alpha :irix6 :x86_64)
      (setf _AnimationID (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _AnimationID (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _AnimationProgress
#+(or :alpha :irix6 :x86_64)
      (setf _AnimationProgress (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _AnimationProgress (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass xr1controllerol::AnimationQuery
  :super ros::object
  :slots ())

(setf (get xr1controllerol::AnimationQuery :md5sum-) "5b670fef922a4d6388f8dea2bff6ef00")
(setf (get xr1controllerol::AnimationQuery :datatype-) "xr1controllerol/AnimationQuery")
(setf (get xr1controllerol::AnimationQuery :request) xr1controllerol::AnimationQueryRequest)
(setf (get xr1controllerol::AnimationQuery :response) xr1controllerol::AnimationQueryResponse)

(defmethod xr1controllerol::AnimationQueryRequest
  (:response () (instance xr1controllerol::AnimationQueryResponse :init)))

(setf (get xr1controllerol::AnimationQueryRequest :md5sum-) "5b670fef922a4d6388f8dea2bff6ef00")
(setf (get xr1controllerol::AnimationQueryRequest :datatype-) "xr1controllerol/AnimationQueryRequest")
(setf (get xr1controllerol::AnimationQueryRequest :definition-)
      "bool isQuery
---
bool inAnimationMode
bool isPlaying
bool hasDefault
bool hasIdle
int64 AnimationType
int64 AnimationID
int64 AnimationProgress

")

(setf (get xr1controllerol::AnimationQueryResponse :md5sum-) "5b670fef922a4d6388f8dea2bff6ef00")
(setf (get xr1controllerol::AnimationQueryResponse :datatype-) "xr1controllerol/AnimationQueryResponse")
(setf (get xr1controllerol::AnimationQueryResponse :definition-)
      "bool isQuery
---
bool inAnimationMode
bool isPlaying
bool hasDefault
bool hasIdle
int64 AnimationType
int64 AnimationID
int64 AnimationProgress

")



(provide :xr1controllerol/AnimationQuery "5b670fef922a4d6388f8dea2bff6ef00")


