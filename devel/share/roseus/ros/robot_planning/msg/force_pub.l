;; Auto-generated. Do not edit!


(when (boundp 'robot_planning::force_pub)
  (if (not (find-package "ROBOT_PLANNING"))
    (make-package "ROBOT_PLANNING"))
  (shadow 'force_pub (find-package "ROBOT_PLANNING")))
(unless (find-package "ROBOT_PLANNING::FORCE_PUB")
  (make-package "ROBOT_PLANNING::FORCE_PUB"))

(in-package "ROS")
;;//! \htmlinclude force_pub.msg.html


(defclass robot_planning::force_pub
  :super ros::object
  :slots (_X _Y _Z _MX _MY _MZ ))

(defmethod robot_planning::force_pub
  (:init
   (&key
    ((:X __X) 0.0)
    ((:Y __Y) 0.0)
    ((:Z __Z) 0.0)
    ((:MX __MX) 0.0)
    ((:MY __MY) 0.0)
    ((:MZ __MZ) 0.0)
    )
   (send-super :init)
   (setq _X (float __X))
   (setq _Y (float __Y))
   (setq _Z (float __Z))
   (setq _MX (float __MX))
   (setq _MY (float __MY))
   (setq _MZ (float __MZ))
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
  (:MX
   (&optional __MX)
   (if __MX (setq _MX __MX)) _MX)
  (:MY
   (&optional __MY)
   (if __MY (setq _MY __MY)) _MY)
  (:MZ
   (&optional __MZ)
   (if __MZ (setq _MZ __MZ)) _MZ)
  (:serialization-length
   ()
   (+
    ;; float64 _X
    8
    ;; float64 _Y
    8
    ;; float64 _Z
    8
    ;; float64 _MX
    8
    ;; float64 _MY
    8
    ;; float64 _MZ
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
     ;; float64 _MX
       (sys::poke _MX (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _MY
       (sys::poke _MY (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _MZ
       (sys::poke _MZ (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
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
   ;; float64 _MX
     (setq _MX (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _MY
     (setq _MY (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _MZ
     (setq _MZ (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get robot_planning::force_pub :md5sum-) "01bd0bd3e5758946edad85592cef21e2")
(setf (get robot_planning::force_pub :datatype-) "robot_planning/force_pub")
(setf (get robot_planning::force_pub :definition-)
      "float64 X
float64 Y
float64 Z
float64 MX
float64 MY
float64 MZ
")



(provide :robot_planning/force_pub "01bd0bd3e5758946edad85592cef21e2")


