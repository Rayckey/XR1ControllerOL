;; Auto-generated. Do not edit!


(when (boundp 'xr1controllerol::AnimationOverwrite)
  (if (not (find-package "XR1CONTROLLEROL"))
    (make-package "XR1CONTROLLEROL"))
  (shadow 'AnimationOverwrite (find-package "XR1CONTROLLEROL")))
(unless (find-package "XR1CONTROLLEROL::ANIMATIONOVERWRITE")
  (make-package "XR1CONTROLLEROL::ANIMATIONOVERWRITE"))
(unless (find-package "XR1CONTROLLEROL::ANIMATIONOVERWRITEREQUEST")
  (make-package "XR1CONTROLLEROL::ANIMATIONOVERWRITEREQUEST"))
(unless (find-package "XR1CONTROLLEROL::ANIMATIONOVERWRITERESPONSE")
  (make-package "XR1CONTROLLEROL::ANIMATIONOVERWRITERESPONSE"))

(in-package "ROS")

(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))




(defclass xr1controllerol::AnimationOverwriteRequest
  :super ros::object
  :slots (_AnimationType _AnimationID _AnimationData ))

(defmethod xr1controllerol::AnimationOverwriteRequest
  (:init
   (&key
    ((:AnimationType __AnimationType) 0)
    ((:AnimationID __AnimationID) 0)
    ((:AnimationData __AnimationData) (instance std_msgs::Float64MultiArray :init))
    )
   (send-super :init)
   (setq _AnimationType (round __AnimationType))
   (setq _AnimationID (round __AnimationID))
   (setq _AnimationData __AnimationData)
   self)
  (:AnimationType
   (&optional __AnimationType)
   (if __AnimationType (setq _AnimationType __AnimationType)) _AnimationType)
  (:AnimationID
   (&optional __AnimationID)
   (if __AnimationID (setq _AnimationID __AnimationID)) _AnimationID)
  (:AnimationData
   (&rest __AnimationData)
   (if (keywordp (car __AnimationData))
       (send* _AnimationData __AnimationData)
     (progn
       (if __AnimationData (setq _AnimationData (car __AnimationData)))
       _AnimationData)))
  (:serialization-length
   ()
   (+
    ;; int32 _AnimationType
    4
    ;; int64 _AnimationID
    8
    ;; std_msgs/Float64MultiArray _AnimationData
    (send _AnimationData :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _AnimationType
       (write-long _AnimationType s)
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
     ;; std_msgs/Float64MultiArray _AnimationData
       (send _AnimationData :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _AnimationType
     (setq _AnimationType (sys::peek buf ptr- :integer)) (incf ptr- 4)
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
   ;; std_msgs/Float64MultiArray _AnimationData
     (send _AnimationData :deserialize buf ptr-) (incf ptr- (send _AnimationData :serialization-length))
   ;;
   self)
  )

(defclass xr1controllerol::AnimationOverwriteResponse
  :super ros::object
  :slots (_isLoaded ))

(defmethod xr1controllerol::AnimationOverwriteResponse
  (:init
   (&key
    ((:isLoaded __isLoaded) nil)
    )
   (send-super :init)
   (setq _isLoaded __isLoaded)
   self)
  (:isLoaded
   (&optional __isLoaded)
   (if __isLoaded (setq _isLoaded __isLoaded)) _isLoaded)
  (:serialization-length
   ()
   (+
    ;; bool _isLoaded
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _isLoaded
       (if _isLoaded (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _isLoaded
     (setq _isLoaded (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass xr1controllerol::AnimationOverwrite
  :super ros::object
  :slots ())

(setf (get xr1controllerol::AnimationOverwrite :md5sum-) "7243db66331b06264c2b466bb4f193aa")
(setf (get xr1controllerol::AnimationOverwrite :datatype-) "xr1controllerol/AnimationOverwrite")
(setf (get xr1controllerol::AnimationOverwrite :request) xr1controllerol::AnimationOverwriteRequest)
(setf (get xr1controllerol::AnimationOverwrite :response) xr1controllerol::AnimationOverwriteResponse)

(defmethod xr1controllerol::AnimationOverwriteRequest
  (:response () (instance xr1controllerol::AnimationOverwriteResponse :init)))

(setf (get xr1controllerol::AnimationOverwriteRequest :md5sum-) "7243db66331b06264c2b466bb4f193aa")
(setf (get xr1controllerol::AnimationOverwriteRequest :datatype-) "xr1controllerol/AnimationOverwriteRequest")
(setf (get xr1controllerol::AnimationOverwriteRequest :definition-)
      "int32 AnimationType
int64 AnimationID
std_msgs/Float64MultiArray AnimationData

================================================================================
MSG: std_msgs/Float64MultiArray
# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
float64[]         data          # array of data


================================================================================
MSG: std_msgs/MultiArrayLayout
# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding elements at front of data

# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
# 
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = \"height\"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = \"width\"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = \"channel\"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
---
bool isLoaded

")

(setf (get xr1controllerol::AnimationOverwriteResponse :md5sum-) "7243db66331b06264c2b466bb4f193aa")
(setf (get xr1controllerol::AnimationOverwriteResponse :datatype-) "xr1controllerol/AnimationOverwriteResponse")
(setf (get xr1controllerol::AnimationOverwriteResponse :definition-)
      "int32 AnimationType
int64 AnimationID
std_msgs/Float64MultiArray AnimationData

================================================================================
MSG: std_msgs/Float64MultiArray
# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
float64[]         data          # array of data


================================================================================
MSG: std_msgs/MultiArrayLayout
# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding elements at front of data

# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
# 
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = \"height\"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = \"width\"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = \"channel\"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
---
bool isLoaded

")



(provide :xr1controllerol/AnimationOverwrite "7243db66331b06264c2b466bb4f193aa")


