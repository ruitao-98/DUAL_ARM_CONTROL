;; Auto-generated. Do not edit!


(when (boundp 'real_robot_control::ori_adj_rec)
  (if (not (find-package "REAL_ROBOT_CONTROL"))
    (make-package "REAL_ROBOT_CONTROL"))
  (shadow 'ori_adj_rec (find-package "REAL_ROBOT_CONTROL")))
(unless (find-package "REAL_ROBOT_CONTROL::ORI_ADJ_REC")
  (make-package "REAL_ROBOT_CONTROL::ORI_ADJ_REC"))

(in-package "ROS")
;;//! \htmlinclude ori_adj_rec.msg.html


(defclass real_robot_control::ori_adj_rec
  :super ros::object
  :slots (_phi _point_num _record_item _Rx _Ry _Rz ))

(defmethod real_robot_control::ori_adj_rec
  (:init
   (&key
    ((:phi __phi) 0)
    ((:point_num __point_num) 0)
    ((:record_item __record_item) 0)
    ((:Rx __Rx) 0.0)
    ((:Ry __Ry) 0.0)
    ((:Rz __Rz) 0.0)
    )
   (send-super :init)
   (setq _phi (round __phi))
   (setq _point_num (round __point_num))
   (setq _record_item (round __record_item))
   (setq _Rx (float __Rx))
   (setq _Ry (float __Ry))
   (setq _Rz (float __Rz))
   self)
  (:phi
   (&optional __phi)
   (if __phi (setq _phi __phi)) _phi)
  (:point_num
   (&optional __point_num)
   (if __point_num (setq _point_num __point_num)) _point_num)
  (:record_item
   (&optional __record_item)
   (if __record_item (setq _record_item __record_item)) _record_item)
  (:Rx
   (&optional __Rx)
   (if __Rx (setq _Rx __Rx)) _Rx)
  (:Ry
   (&optional __Ry)
   (if __Ry (setq _Ry __Ry)) _Ry)
  (:Rz
   (&optional __Rz)
   (if __Rz (setq _Rz __Rz)) _Rz)
  (:serialization-length
   ()
   (+
    ;; int32 _phi
    4
    ;; int32 _point_num
    4
    ;; int32 _record_item
    4
    ;; float64 _Rx
    8
    ;; float64 _Ry
    8
    ;; float64 _Rz
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _phi
       (write-long _phi s)
     ;; int32 _point_num
       (write-long _point_num s)
     ;; int32 _record_item
       (write-long _record_item s)
     ;; float64 _Rx
       (sys::poke _Rx (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _Ry
       (sys::poke _Ry (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _Rz
       (sys::poke _Rz (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _phi
     (setq _phi (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _point_num
     (setq _point_num (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _record_item
     (setq _record_item (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _Rx
     (setq _Rx (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _Ry
     (setq _Ry (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _Rz
     (setq _Rz (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get real_robot_control::ori_adj_rec :md5sum-) "41fd7db046eaccfb5112edd2ddb827ba")
(setf (get real_robot_control::ori_adj_rec :datatype-) "real_robot_control/ori_adj_rec")
(setf (get real_robot_control::ori_adj_rec :definition-)
      "int32 phi
int32 point_num
int32 record_item
float64 Rx
float64 Ry
float64 Rz
")



(provide :real_robot_control/ori_adj_rec "41fd7db046eaccfb5112edd2ddb827ba")


