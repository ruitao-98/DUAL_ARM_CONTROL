; Auto-generated. Do not edit!


(cl:in-package real_robot_control-msg)


;//! \htmlinclude ori_adj_rec.msg.html

(cl:defclass <ori_adj_rec> (roslisp-msg-protocol:ros-message)
  ((phi
    :reader phi
    :initarg :phi
    :type cl:integer
    :initform 0)
   (point_num
    :reader point_num
    :initarg :point_num
    :type cl:integer
    :initform 0)
   (record_item
    :reader record_item
    :initarg :record_item
    :type cl:integer
    :initform 0)
   (Rx
    :reader Rx
    :initarg :Rx
    :type cl:float
    :initform 0.0)
   (Ry
    :reader Ry
    :initarg :Ry
    :type cl:float
    :initform 0.0)
   (Rz
    :reader Rz
    :initarg :Rz
    :type cl:float
    :initform 0.0))
)

(cl:defclass ori_adj_rec (<ori_adj_rec>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ori_adj_rec>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ori_adj_rec)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name real_robot_control-msg:<ori_adj_rec> is deprecated: use real_robot_control-msg:ori_adj_rec instead.")))

(cl:ensure-generic-function 'phi-val :lambda-list '(m))
(cl:defmethod phi-val ((m <ori_adj_rec>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader real_robot_control-msg:phi-val is deprecated.  Use real_robot_control-msg:phi instead.")
  (phi m))

(cl:ensure-generic-function 'point_num-val :lambda-list '(m))
(cl:defmethod point_num-val ((m <ori_adj_rec>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader real_robot_control-msg:point_num-val is deprecated.  Use real_robot_control-msg:point_num instead.")
  (point_num m))

(cl:ensure-generic-function 'record_item-val :lambda-list '(m))
(cl:defmethod record_item-val ((m <ori_adj_rec>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader real_robot_control-msg:record_item-val is deprecated.  Use real_robot_control-msg:record_item instead.")
  (record_item m))

(cl:ensure-generic-function 'Rx-val :lambda-list '(m))
(cl:defmethod Rx-val ((m <ori_adj_rec>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader real_robot_control-msg:Rx-val is deprecated.  Use real_robot_control-msg:Rx instead.")
  (Rx m))

(cl:ensure-generic-function 'Ry-val :lambda-list '(m))
(cl:defmethod Ry-val ((m <ori_adj_rec>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader real_robot_control-msg:Ry-val is deprecated.  Use real_robot_control-msg:Ry instead.")
  (Ry m))

(cl:ensure-generic-function 'Rz-val :lambda-list '(m))
(cl:defmethod Rz-val ((m <ori_adj_rec>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader real_robot_control-msg:Rz-val is deprecated.  Use real_robot_control-msg:Rz instead.")
  (Rz m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ori_adj_rec>) ostream)
  "Serializes a message object of type '<ori_adj_rec>"
  (cl:let* ((signed (cl:slot-value msg 'phi)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'point_num)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'record_item)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'Rx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'Ry))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'Rz))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ori_adj_rec>) istream)
  "Deserializes a message object of type '<ori_adj_rec>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'phi) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'point_num) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'record_item) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Rx) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Ry) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Rz) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ori_adj_rec>)))
  "Returns string type for a message object of type '<ori_adj_rec>"
  "real_robot_control/ori_adj_rec")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ori_adj_rec)))
  "Returns string type for a message object of type 'ori_adj_rec"
  "real_robot_control/ori_adj_rec")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ori_adj_rec>)))
  "Returns md5sum for a message object of type '<ori_adj_rec>"
  "41fd7db046eaccfb5112edd2ddb827ba")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ori_adj_rec)))
  "Returns md5sum for a message object of type 'ori_adj_rec"
  "41fd7db046eaccfb5112edd2ddb827ba")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ori_adj_rec>)))
  "Returns full string definition for message of type '<ori_adj_rec>"
  (cl:format cl:nil "int32 phi~%int32 point_num~%int32 record_item~%float64 Rx~%float64 Ry~%float64 Rz~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ori_adj_rec)))
  "Returns full string definition for message of type 'ori_adj_rec"
  (cl:format cl:nil "int32 phi~%int32 point_num~%int32 record_item~%float64 Rx~%float64 Ry~%float64 Rz~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ori_adj_rec>))
  (cl:+ 0
     4
     4
     4
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ori_adj_rec>))
  "Converts a ROS message object to a list"
  (cl:list 'ori_adj_rec
    (cl:cons ':phi (phi msg))
    (cl:cons ':point_num (point_num msg))
    (cl:cons ':record_item (record_item msg))
    (cl:cons ':Rx (Rx msg))
    (cl:cons ':Ry (Ry msg))
    (cl:cons ':Rz (Rz msg))
))
