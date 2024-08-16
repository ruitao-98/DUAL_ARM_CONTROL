; Auto-generated. Do not edit!


(cl:in-package real_robot_control-msg)


;//! \htmlinclude orientation_pub.msg.html

(cl:defclass <orientation_pub> (roslisp-msg-protocol:ros-message)
  ((phi
    :reader phi
    :initarg :phi
    :type cl:float
    :initform 0.0))
)

(cl:defclass orientation_pub (<orientation_pub>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <orientation_pub>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'orientation_pub)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name real_robot_control-msg:<orientation_pub> is deprecated: use real_robot_control-msg:orientation_pub instead.")))

(cl:ensure-generic-function 'phi-val :lambda-list '(m))
(cl:defmethod phi-val ((m <orientation_pub>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader real_robot_control-msg:phi-val is deprecated.  Use real_robot_control-msg:phi instead.")
  (phi m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <orientation_pub>) ostream)
  "Serializes a message object of type '<orientation_pub>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'phi))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <orientation_pub>) istream)
  "Deserializes a message object of type '<orientation_pub>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'phi) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<orientation_pub>)))
  "Returns string type for a message object of type '<orientation_pub>"
  "real_robot_control/orientation_pub")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'orientation_pub)))
  "Returns string type for a message object of type 'orientation_pub"
  "real_robot_control/orientation_pub")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<orientation_pub>)))
  "Returns md5sum for a message object of type '<orientation_pub>"
  "ea4d94f9452111dac782c21036db1934")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'orientation_pub)))
  "Returns md5sum for a message object of type 'orientation_pub"
  "ea4d94f9452111dac782c21036db1934")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<orientation_pub>)))
  "Returns full string definition for message of type '<orientation_pub>"
  (cl:format cl:nil "float64 phi~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'orientation_pub)))
  "Returns full string definition for message of type 'orientation_pub"
  (cl:format cl:nil "float64 phi~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <orientation_pub>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <orientation_pub>))
  "Converts a ROS message object to a list"
  (cl:list 'orientation_pub
    (cl:cons ':phi (phi msg))
))
