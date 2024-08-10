;; Auto-generated. Do not edit!


(when (boundp 'real_robot_control::screwsrv)
  (if (not (find-package "REAL_ROBOT_CONTROL"))
    (make-package "REAL_ROBOT_CONTROL"))
  (shadow 'screwsrv (find-package "REAL_ROBOT_CONTROL")))
(unless (find-package "REAL_ROBOT_CONTROL::SCREWSRV")
  (make-package "REAL_ROBOT_CONTROL::SCREWSRV"))
(unless (find-package "REAL_ROBOT_CONTROL::SCREWSRVREQUEST")
  (make-package "REAL_ROBOT_CONTROL::SCREWSRVREQUEST"))
(unless (find-package "REAL_ROBOT_CONTROL::SCREWSRVRESPONSE")
  (make-package "REAL_ROBOT_CONTROL::SCREWSRVRESPONSE"))

(in-package "ROS")





(defclass real_robot_control::screwsrvRequest
  :super ros::object
  :slots (_num ))

(defmethod real_robot_control::screwsrvRequest
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

(defclass real_robot_control::screwsrvResponse
  :super ros::object
  :slots (_result ))

(defmethod real_robot_control::screwsrvResponse
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

(defclass real_robot_control::screwsrv
  :super ros::object
  :slots ())

(setf (get real_robot_control::screwsrv :md5sum-) "7b18929c222448a67fda0cd87f44304d")
(setf (get real_robot_control::screwsrv :datatype-) "real_robot_control/screwsrv")
(setf (get real_robot_control::screwsrv :request) real_robot_control::screwsrvRequest)
(setf (get real_robot_control::screwsrv :response) real_robot_control::screwsrvResponse)

(defmethod real_robot_control::screwsrvRequest
  (:response () (instance real_robot_control::screwsrvResponse :init)))

(setf (get real_robot_control::screwsrvRequest :md5sum-) "7b18929c222448a67fda0cd87f44304d")
(setf (get real_robot_control::screwsrvRequest :datatype-) "real_robot_control/screwsrvRequest")
(setf (get real_robot_control::screwsrvRequest :definition-)
      "#目标值
int32 num
---
#最终结果
int32 result

")

(setf (get real_robot_control::screwsrvResponse :md5sum-) "7b18929c222448a67fda0cd87f44304d")
(setf (get real_robot_control::screwsrvResponse :datatype-) "real_robot_control/screwsrvResponse")
(setf (get real_robot_control::screwsrvResponse :definition-)
      "#目标值
int32 num
---
#最终结果
int32 result

")



(provide :real_robot_control/screwsrv "7b18929c222448a67fda0cd87f44304d")


