;; Auto-generated. Do not edit!


(when (boundp 'real_robot_control::pose_pub)
  (if (not (find-package "REAL_ROBOT_CONTROL"))
    (make-package "REAL_ROBOT_CONTROL"))
  (shadow 'pose_pub (find-package "REAL_ROBOT_CONTROL")))
(unless (find-package "REAL_ROBOT_CONTROL::POSE_PUB")
  (make-package "REAL_ROBOT_CONTROL::POSE_PUB"))

(in-package "ROS")
;;//! \htmlinclude pose_pub.msg.html


(defclass real_robot_control::pose_pub
  :super ros::object
  :slots (_X _Y _Z _RX _RY _RZ _FX _FY _FZ _theta ))

(defmethod real_robot_control::pose_pub
  (:init
   (&key
    ((:X __X) 0.0)
    ((:Y __Y) 0.0)
    ((:Z __Z) 0.0)
    ((:RX __RX) 0.0)
    ((:RY __RY) 0.0)
    ((:RZ __RZ) 0.0)
    ((:FX __FX) 0.0)
    ((:FY __FY) 0.0)
    ((:FZ __FZ) 0.0)
    ((:theta __theta) 0.0)
    )
   (send-super :init)
   (setq _X (float __X))
   (setq _Y (float __Y))
   (setq _Z (float __Z))
   (setq _RX (float __RX))
   (setq _RY (float __RY))
   (setq _RZ (float __RZ))
   (setq _FX (float __FX))
   (setq _FY (float __FY))
   (setq _FZ (float __FZ))
   (setq _theta (float __theta))
   self)
  (:X
   (&optional __X)
   (if __X (setq _X __X)) _X)
  (:Y
   (&optional __Y)
   (if __Y (setq _Y __Y)) _Y)
  (:Z
   (&optional __Z)
   (if __Z (setq _Z __Z)) _Z)
  (:RX
   (&optional __RX)
   (if __RX (setq _RX __RX)) _RX)
  (:RY
   (&optional __RY)
   (if __RY (setq _RY __RY)) _RY)
  (:RZ
   (&optional __RZ)
   (if __RZ (setq _RZ __RZ)) _RZ)
  (:FX
   (&optional __FX)
   (if __FX (setq _FX __FX)) _FX)
  (:FY
   (&optional __FY)
   (if __FY (setq _FY __FY)) _FY)
  (:FZ
   (&optional __FZ)
   (if __FZ (setq _FZ __FZ)) _FZ)
  (:theta
   (&optional __theta)
   (if __theta (setq _theta __theta)) _theta)
  (:serialization-length
   ()
   (+
    ;; float64 _X
    8
    ;; float64 _Y
    8
    ;; float64 _Z
    8
    ;; float64 _RX
    8
    ;; float64 _RY
    8
    ;; float64 _RZ
    8
    ;; float64 _FX
    8
    ;; float64 _FY
    8
    ;; float64 _FZ
    8
    ;; float64 _theta
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _X
       (sys::poke _X (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _Y
       (sys::poke _Y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _Z
       (sys::poke _Z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _RX
       (sys::poke _RX (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _RY
       (sys::poke _RY (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _RZ
       (sys::poke _RZ (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _FX
       (sys::poke _FX (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _FY
       (sys::poke _FY (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _FZ
       (sys::poke _FZ (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _theta
       (sys::poke _theta (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _X
     (setq _X (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _Y
     (setq _Y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _Z
     (setq _Z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _RX
     (setq _RX (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _RY
     (setq _RY (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _RZ
     (setq _RZ (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _FX
     (setq _FX (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _FY
     (setq _FY (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _FZ
     (setq _FZ (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _theta
     (setq _theta (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get real_robot_control::pose_pub :md5sum-) "d20e99fa36280381d3010eacb694aaa0")
(setf (get real_robot_control::pose_pub :datatype-) "real_robot_control/pose_pub")
(setf (get real_robot_control::pose_pub :definition-)
      "float64 X
float64 Y
float64 Z
float64 RX
float64 RY
float64 RZ
float64 FX
float64 FY
float64 FZ 
float64 theta

")



(provide :real_robot_control/pose_pub "d20e99fa36280381d3010eacb694aaa0")


