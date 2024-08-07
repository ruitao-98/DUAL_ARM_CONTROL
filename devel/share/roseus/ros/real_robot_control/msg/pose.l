;; Auto-generated. Do not edit!


(when (boundp 'real_robot_control::pose)
  (if (not (find-package "REAL_ROBOT_CONTROL"))
    (make-package "REAL_ROBOT_CONTROL"))
  (shadow 'pose (find-package "REAL_ROBOT_CONTROL")))
(unless (find-package "REAL_ROBOT_CONTROL::POSE")
  (make-package "REAL_ROBOT_CONTROL::POSE"))

(in-package "ROS")
;;//! \htmlinclude pose.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass real_robot_control::pose
  :super ros::object
  :slots (_matrices ))

(defmethod real_robot_control::pose
  (:init
   (&key
    ((:matrices __matrices) ())
    )
   (send-super :init)
   (setq _matrices __matrices)
   self)
  (:matrices
   (&rest __matrices)
   (if (keywordp (car __matrices))
       (send* _matrices __matrices)
     (progn
       (if __matrices (setq _matrices (car __matrices)))
       _matrices)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Float32MultiArray[] _matrices
    (apply #'+ (send-all _matrices :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Float32MultiArray[] _matrices
     (write-long (length _matrices) s)
     (dolist (elem _matrices)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Float32MultiArray[] _matrices
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _matrices (let (r) (dotimes (i n) (push (instance std_msgs::Float32MultiArray :init) r)) r))
     (dolist (elem- _matrices)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get real_robot_control::pose :md5sum-) "04c3645e7964b08a28c0477b52d83e5b")
(setf (get real_robot_control::pose :datatype-) "real_robot_control/pose")
(setf (get real_robot_control::pose :definition-)
      "std_msgs/Float32MultiArray[] matrices
================================================================================
MSG: std_msgs/Float32MultiArray
# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
float32[]         data          # array of data


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
uint32 stride  # stride of given dimension
")



(provide :real_robot_control/pose "04c3645e7964b08a28c0477b52d83e5b")


