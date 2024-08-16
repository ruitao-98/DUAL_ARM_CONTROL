;; Auto-generated. Do not edit!


(when (boundp 'real_robot_control::force_pos_pub)
  (if (not (find-package "REAL_ROBOT_CONTROL"))
    (make-package "REAL_ROBOT_CONTROL"))
  (shadow 'force_pos_pub (find-package "REAL_ROBOT_CONTROL")))
(unless (find-package "REAL_ROBOT_CONTROL::FORCE_POS_PUB")
  (make-package "REAL_ROBOT_CONTROL::FORCE_POS_PUB"))

(in-package "ROS")
;;//! \htmlinclude force_pos_pub.msg.html


(defclass real_robot_control::force_pos_pub
  :super ros::object
  :slots (_FX _FY _FZ _X _Y _Z _theta ))

(defmethod real_robot_control::force_pos_pub
  (:init
   (&key
    ((:FX __FX) 0.0)
    ((:FY __FY) 0.0)
    ((:FZ __FZ) 0.0)
    ((:X __X) 0.0)
    ((:Y __Y) 0.0)
    ((:Z __Z) 0.0)
    ((:theta __theta) 0.0)
    )
   (send-super :init)
   (setq _FX (float __FX))
   (setq _FY (float __FY))
   (setq _FZ (float __FZ))
   (setq _X (float __X))
   (setq _Y (float __Y))
   (setq _Z (float __Z))
   (setq _theta (float __theta))
   self)
  (:FX
   (&optional __FX)
   (if __FX (setq _FX __FX)) _FX)
  (:FY
   (&optional __FY)
   (if __FY (setq _FY __FY)) _FY)
  (:FZ
   (&optional __FZ)
   (if __FZ (setq _FZ __FZ)) _FZ)
  (:X
   (&optional __X)
   (if __X (setq _X __X)) _X)
  (:Y
   (&optional __Y)
   (if __Y (setq _Y __Y)) _Y)
  (:Z
   (&optional __Z)
   (if __Z (setq _Z __Z)) _Z)
  (:theta
   (&optional __theta)
   (if __theta (setq _theta __theta)) _theta)
  (:serialization-length
   ()
   (+
    ;; float64 _FX
    8
    ;; float64 _FY
    8
    ;; float64 _FZ
    8
    ;; float64 _X
    8
    ;; float64 _Y
    8
    ;; float64 _Z
    8
    ;; float64 _theta
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _FX
       (sys::poke _FX (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _FY
       (sys::poke _FY (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _FZ
       (sys::poke _FZ (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _X
       (sys::poke _X (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _Y
       (sys::poke _Y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _Z
       (sys::poke _Z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _theta
       (sys::poke _theta (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _FX
     (setq _FX (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _FY
     (setq _FY (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _FZ
     (setq _FZ (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _X
     (setq _X (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _Y
     (setq _Y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _Z
     (setq _Z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _theta
     (setq _theta (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get real_robot_control::force_pos_pub :md5sum-) "602988f5260503914aa61c83fb41c15c")
(setf (get real_robot_control::force_pos_pub :datatype-) "real_robot_control/force_pos_pub")
(setf (get real_robot_control::force_pos_pub :definition-)
      "float64 FX
float64 FY
float64 FZ
float64 X
float64 Y
float64 Z
float64 theta
")



(provide :real_robot_control/force_pos_pub "602988f5260503914aa61c83fb41c15c")


