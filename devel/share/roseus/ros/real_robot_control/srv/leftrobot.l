;; Auto-generated. Do not edit!


(when (boundp 'real_robot_control::leftrobot)
  (if (not (find-package "REAL_ROBOT_CONTROL"))
    (make-package "REAL_ROBOT_CONTROL"))
  (shadow 'leftrobot (find-package "REAL_ROBOT_CONTROL")))
(unless (find-package "REAL_ROBOT_CONTROL::LEFTROBOT")
  (make-package "REAL_ROBOT_CONTROL::LEFTROBOT"))
(unless (find-package "REAL_ROBOT_CONTROL::LEFTROBOTREQUEST")
  (make-package "REAL_ROBOT_CONTROL::LEFTROBOTREQUEST"))
(unless (find-package "REAL_ROBOT_CONTROL::LEFTROBOTRESPONSE")
  (make-package "REAL_ROBOT_CONTROL::LEFTROBOTRESPONSE"))

(in-package "ROS")





(defclass real_robot_control::leftrobotRequest
  :super ros::object
  :slots (_num ))

(defmethod real_robot_control::leftrobotRequest
  (:init
   (&key
    ((:num __num) 0)
    )
   (send-super :init)
   (setq _num (round __num))
   self)
  (:num
   (&optional __num)
   (if __num (setq _num __num)) _num)
  (:serialization-length
   ()
   (+
    ;; int32 _num
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _num
       (write-long _num s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _num
     (setq _num (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass real_robot_control::leftrobotResponse
  :super ros::object
  :slots (_result ))

(defmethod real_robot_control::leftrobotResponse
  (:init
   (&key
    ((:result __result) 0)
    )
   (send-super :init)
   (setq _result (round __result))
   self)
  (:result
   (&optional __result)
   (if __result (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; int32 _result
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _result
       (write-long _result s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _result
     (setq _result (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass real_robot_control::leftrobot
  :super ros::object
  :slots ())

(setf (get real_robot_control::leftrobot :md5sum-) "7b18929c222448a67fda0cd87f44304d")
(setf (get real_robot_control::leftrobot :datatype-) "real_robot_control/leftrobot")
(setf (get real_robot_control::leftrobot :request) real_robot_control::leftrobotRequest)
(setf (get real_robot_control::leftrobot :response) real_robot_control::leftrobotResponse)

(defmethod real_robot_control::leftrobotRequest
  (:response () (instance real_robot_control::leftrobotResponse :init)))

(setf (get real_robot_control::leftrobotRequest :md5sum-) "7b18929c222448a67fda0cd87f44304d")
(setf (get real_robot_control::leftrobotRequest :datatype-) "real_robot_control/leftrobotRequest")
(setf (get real_robot_control::leftrobotRequest :definition-)
      "#目标值
int32 num
---
#最终结果
int32 result
")

(setf (get real_robot_control::leftrobotResponse :md5sum-) "7b18929c222448a67fda0cd87f44304d")
(setf (get real_robot_control::leftrobotResponse :datatype-) "real_robot_control/leftrobotResponse")
(setf (get real_robot_control::leftrobotResponse :definition-)
      "#目标值
int32 num
---
#最终结果
int32 result
")



(provide :real_robot_control/leftrobot "7b18929c222448a67fda0cd87f44304d")


