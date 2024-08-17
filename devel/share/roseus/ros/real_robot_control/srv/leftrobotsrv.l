;; Auto-generated. Do not edit!


(when (boundp 'real_robot_control::leftrobotsrv)
  (if (not (find-package "REAL_ROBOT_CONTROL"))
    (make-package "REAL_ROBOT_CONTROL"))
  (shadow 'leftrobotsrv (find-package "REAL_ROBOT_CONTROL")))
(unless (find-package "REAL_ROBOT_CONTROL::LEFTROBOTSRV")
  (make-package "REAL_ROBOT_CONTROL::LEFTROBOTSRV"))
(unless (find-package "REAL_ROBOT_CONTROL::LEFTROBOTSRVREQUEST")
  (make-package "REAL_ROBOT_CONTROL::LEFTROBOTSRVREQUEST"))
(unless (find-package "REAL_ROBOT_CONTROL::LEFTROBOTSRVRESPONSE")
  (make-package "REAL_ROBOT_CONTROL::LEFTROBOTSRVRESPONSE"))

(in-package "ROS")





(defclass real_robot_control::leftrobotsrvRequest
  :super ros::object
  :slots (_num ))

(defmethod real_robot_control::leftrobotsrvRequest
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

(defclass real_robot_control::leftrobotsrvResponse
  :super ros::object
  :slots (_result ))

(defmethod real_robot_control::leftrobotsrvResponse
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

(defclass real_robot_control::leftrobotsrv
  :super ros::object
  :slots ())

(setf (get real_robot_control::leftrobotsrv :md5sum-) "7b18929c222448a67fda0cd87f44304d")
(setf (get real_robot_control::leftrobotsrv :datatype-) "real_robot_control/leftrobotsrv")
(setf (get real_robot_control::leftrobotsrv :request) real_robot_control::leftrobotsrvRequest)
(setf (get real_robot_control::leftrobotsrv :response) real_robot_control::leftrobotsrvResponse)

(defmethod real_robot_control::leftrobotsrvRequest
  (:response () (instance real_robot_control::leftrobotsrvResponse :init)))

(setf (get real_robot_control::leftrobotsrvRequest :md5sum-) "7b18929c222448a67fda0cd87f44304d")
(setf (get real_robot_control::leftrobotsrvRequest :datatype-) "real_robot_control/leftrobotsrvRequest")
(setf (get real_robot_control::leftrobotsrvRequest :definition-)
      "#目标值
int32 num
---
#最终结果
int32 result
")

(setf (get real_robot_control::leftrobotsrvResponse :md5sum-) "7b18929c222448a67fda0cd87f44304d")
(setf (get real_robot_control::leftrobotsrvResponse :datatype-) "real_robot_control/leftrobotsrvResponse")
(setf (get real_robot_control::leftrobotsrvResponse :definition-)
      "#目标值
int32 num
---
#最终结果
int32 result
")



(provide :real_robot_control/leftrobotsrv "7b18929c222448a67fda0cd87f44304d")


