;; Auto-generated. Do not edit!


(when (boundp 'real_robot_control::orientation_pub)
  (if (not (find-package "REAL_ROBOT_CONTROL"))
    (make-package "REAL_ROBOT_CONTROL"))
  (shadow 'orientation_pub (find-package "REAL_ROBOT_CONTROL")))
(unless (find-package "REAL_ROBOT_CONTROL::ORIENTATION_PUB")
  (make-package "REAL_ROBOT_CONTROL::ORIENTATION_PUB"))

(in-package "ROS")
;;//! \htmlinclude orientation_pub.msg.html


(defclass real_robot_control::orientation_pub
  :super ros::object
  :slots (_phi ))

(defmethod real_robot_control::orientation_pub
  (:init
   (&key
    ((:phi __phi) 0.0)
    )
   (send-super :init)
   (setq _phi (float __phi))
   self)
  (:phi
   (&optional __phi)
   (if __phi (setq _phi __phi)) _phi)
  (:serialization-length
   ()
   (+
    ;; float64 _phi
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _phi
       (sys::poke _phi (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _phi
     (setq _phi (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get real_robot_control::orientation_pub :md5sum-) "ea4d94f9452111dac782c21036db1934")
(setf (get real_robot_control::orientation_pub :datatype-) "real_robot_control/orientation_pub")
(setf (get real_robot_control::orientation_pub :definition-)
      "float64 phi
")



(provide :real_robot_control/orientation_pub "ea4d94f9452111dac782c21036db1934")


