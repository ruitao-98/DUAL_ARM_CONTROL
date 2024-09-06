; Auto-generated. Do not edit!


(cl:in-package real_robot_control-msg)


;//! \htmlinclude width_pub.msg.html

(cl:defclass <width_pub> (roslisp-msg-protocol:ros-message)
  ((width
    :reader width
    :initarg :width
    :type cl:float
    :initform 0.0))
)

(cl:defclass width_pub (<width_pub>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <width_pub>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'width_pub)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name real_robot_control-msg:<width_pub> is deprecated: use real_robot_control-msg:width_pub instead.")))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <width_pub>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader real_robot_control-msg:width-val is deprecated.  Use real_robot_control-msg:width instead.")
  (width m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <width_pub>) ostream)
  "Serializes a message object of type '<width_pub>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'width))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <width_pub>) istream)
  "Deserializes a message object of type '<width_pub>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'width) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<width_pub>)))
  "Returns string type for a message object of type '<width_pub>"
  "real_robot_control/width_pub")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'width_pub)))
  "Returns string type for a message object of type 'width_pub"
  "real_robot_control/width_pub")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<width_pub>)))
  "Returns md5sum for a message object of type '<width_pub>"
  "a334e8a8f988e31f739167a339fc51af")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'width_pub)))
  "Returns md5sum for a message object of type 'width_pub"
  "a334e8a8f988e31f739167a339fc51af")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<width_pub>)))
  "Returns full string definition for message of type '<width_pub>"
  (cl:format cl:nil "float64 width~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'width_pub)))
  "Returns full string definition for message of type 'width_pub"
  (cl:format cl:nil "float64 width~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <width_pub>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <width_pub>))
  "Converts a ROS message object to a list"
  (cl:list 'width_pub
    (cl:cons ':width (width msg))
))
