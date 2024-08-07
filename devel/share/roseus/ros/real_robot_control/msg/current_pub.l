;; Auto-generated. Do not edit!


(when (boundp 'real_robot_control::current_pub)
  (if (not (find-package "REAL_ROBOT_CONTROL"))
    (make-package "REAL_ROBOT_CONTROL"))
  (shadow 'current_pub (find-package "REAL_ROBOT_CONTROL")))
(unless (find-package "REAL_ROBOT_CONTROL::CURRENT_PUB")
  (make-package "REAL_ROBOT_CONTROL::CURRENT_PUB"))

(in-package "ROS")
;;//! \htmlinclude current_pub.msg.html


(defclass real_robot_control::current_pub
  :super ros::object
  :slots (_current ))

(defmethod real_robot_control::current_pub
  (:init
   (&key
    ((:current __current) 0.0)
    )
   (send-super :init)
   (setq _current (float __current))
   self)
  (:current
   (&optional __current)
   (if __current (setq _current __current)) _current)
  (:serialization-length
   ()
   (+
    ;; float64 _current
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _current
       (sys::poke _current (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _current
     (setq _current (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get real_robot_control::current_pub :md5sum-) "ca4d0ba43a70fe6e37b76accdbf3ef40")
(setf (get real_robot_control::current_pub :datatype-) "real_robot_control/current_pub")
(setf (get real_robot_control::current_pub :definition-)
      "float64 current
")



(provide :real_robot_control/current_pub "ca4d0ba43a70fe6e37b76accdbf3ef40")


