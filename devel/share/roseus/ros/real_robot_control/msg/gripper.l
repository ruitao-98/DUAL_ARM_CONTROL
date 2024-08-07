;; Auto-generated. Do not edit!


(when (boundp 'real_robot_control::gripper)
  (if (not (find-package "REAL_ROBOT_CONTROL"))
    (make-package "REAL_ROBOT_CONTROL"))
  (shadow 'gripper (find-package "REAL_ROBOT_CONTROL")))
(unless (find-package "REAL_ROBOT_CONTROL::GRIPPER")
  (make-package "REAL_ROBOT_CONTROL::GRIPPER"))

(in-package "ROS")
;;//! \htmlinclude gripper.msg.html


(defclass real_robot_control::gripper
  :super ros::object
  :slots (_open ))

(defmethod real_robot_control::gripper
  (:init
   (&key
    ((:open __open) 0.0)
    )
   (send-super :init)
   (setq _open (float __open))
   self)
  (:open
   (&optional __open)
   (if __open (setq _open __open)) _open)
  (:serialization-length
   ()
   (+
    ;; float64 _open
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _open
       (sys::poke _open (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _open
     (setq _open (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get real_robot_control::gripper :md5sum-) "1cd3c8c0899095b58f167c1f2be4d95a")
(setf (get real_robot_control::gripper :datatype-) "real_robot_control/gripper")
(setf (get real_robot_control::gripper :definition-)
      "float64 open
")



(provide :real_robot_control/gripper "1cd3c8c0899095b58f167c1f2be4d95a")


