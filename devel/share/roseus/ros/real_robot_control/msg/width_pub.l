;; Auto-generated. Do not edit!


(when (boundp 'real_robot_control::width_pub)
  (if (not (find-package "REAL_ROBOT_CONTROL"))
    (make-package "REAL_ROBOT_CONTROL"))
  (shadow 'width_pub (find-package "REAL_ROBOT_CONTROL")))
(unless (find-package "REAL_ROBOT_CONTROL::WIDTH_PUB")
  (make-package "REAL_ROBOT_CONTROL::WIDTH_PUB"))

(in-package "ROS")
;;//! \htmlinclude width_pub.msg.html


(defclass real_robot_control::width_pub
  :super ros::object
  :slots (_width ))

(defmethod real_robot_control::width_pub
  (:init
   (&key
    ((:width __width) 0.0)
    )
   (send-super :init)
   (setq _width (float __width))
   self)
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:serialization-length
   ()
   (+
    ;; float64 _width
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _width
       (sys::poke _width (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _width
     (setq _width (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get real_robot_control::width_pub :md5sum-) "a334e8a8f988e31f739167a339fc51af")
(setf (get real_robot_control::width_pub :datatype-) "real_robot_control/width_pub")
(setf (get real_robot_control::width_pub :definition-)
      "float64 width
")



(provide :real_robot_control/width_pub "a334e8a8f988e31f739167a339fc51af")


