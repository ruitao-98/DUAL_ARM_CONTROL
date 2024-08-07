; Auto-generated. Do not edit!


(cl:in-package real_robot_control-msg)


;//! \htmlinclude current_pub.msg.html

(cl:defclass <current_pub> (roslisp-msg-protocol:ros-message)
  ((current
    :reader current
    :initarg :current
    :type cl:float
    :initform 0.0))
)

(cl:defclass current_pub (<current_pub>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <current_pub>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'current_pub)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name real_robot_control-msg:<current_pub> is deprecated: use real_robot_control-msg:current_pub instead.")))

(cl:ensure-generic-function 'current-val :lambda-list '(m))
(cl:defmethod current-val ((m <current_pub>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader real_robot_control-msg:current-val is deprecated.  Use real_robot_control-msg:current instead.")
  (current m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <current_pub>) ostream)
  "Serializes a message object of type '<current_pub>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'current))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <current_pub>) istream)
  "Deserializes a message object of type '<current_pub>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<current_pub>)))
  "Returns string type for a message object of type '<current_pub>"
  "real_robot_control/current_pub")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'current_pub)))
  "Returns string type for a message object of type 'current_pub"
  "real_robot_control/current_pub")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<current_pub>)))
  "Returns md5sum for a message object of type '<current_pub>"
  "ca4d0ba43a70fe6e37b76accdbf3ef40")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'current_pub)))
  "Returns md5sum for a message object of type 'current_pub"
  "ca4d0ba43a70fe6e37b76accdbf3ef40")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<current_pub>)))
  "Returns full string definition for message of type '<current_pub>"
  (cl:format cl:nil "float64 current~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'current_pub)))
  "Returns full string definition for message of type 'current_pub"
  (cl:format cl:nil "float64 current~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <current_pub>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <current_pub>))
  "Converts a ROS message object to a list"
  (cl:list 'current_pub
    (cl:cons ':current (current msg))
))
