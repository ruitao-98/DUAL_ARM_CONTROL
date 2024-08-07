; Auto-generated. Do not edit!


(cl:in-package real_robot_control-msg)


;//! \htmlinclude gripper.msg.html

(cl:defclass <gripper> (roslisp-msg-protocol:ros-message)
  ((open
    :reader open
    :initarg :open
    :type cl:float
    :initform 0.0))
)

(cl:defclass gripper (<gripper>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gripper>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gripper)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name real_robot_control-msg:<gripper> is deprecated: use real_robot_control-msg:gripper instead.")))

(cl:ensure-generic-function 'open-val :lambda-list '(m))
(cl:defmethod open-val ((m <gripper>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader real_robot_control-msg:open-val is deprecated.  Use real_robot_control-msg:open instead.")
  (open m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gripper>) ostream)
  "Serializes a message object of type '<gripper>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'open))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gripper>) istream)
  "Deserializes a message object of type '<gripper>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'open) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gripper>)))
  "Returns string type for a message object of type '<gripper>"
  "real_robot_control/gripper")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gripper)))
  "Returns string type for a message object of type 'gripper"
  "real_robot_control/gripper")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gripper>)))
  "Returns md5sum for a message object of type '<gripper>"
  "1cd3c8c0899095b58f167c1f2be4d95a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gripper)))
  "Returns md5sum for a message object of type 'gripper"
  "1cd3c8c0899095b58f167c1f2be4d95a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gripper>)))
  "Returns full string definition for message of type '<gripper>"
  (cl:format cl:nil "float64 open~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gripper)))
  "Returns full string definition for message of type 'gripper"
  (cl:format cl:nil "float64 open~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gripper>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gripper>))
  "Converts a ROS message object to a list"
  (cl:list 'gripper
    (cl:cons ':open (open msg))
))
