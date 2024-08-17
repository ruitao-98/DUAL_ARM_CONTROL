; Auto-generated. Do not edit!


(cl:in-package real_robot_control-srv)


;//! \htmlinclude leftrobot-request.msg.html

(cl:defclass <leftrobot-request> (roslisp-msg-protocol:ros-message)
  ((num
    :reader num
    :initarg :num
    :type cl:integer
    :initform 0))
)

(cl:defclass leftrobot-request (<leftrobot-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <leftrobot-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'leftrobot-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name real_robot_control-srv:<leftrobot-request> is deprecated: use real_robot_control-srv:leftrobot-request instead.")))

(cl:ensure-generic-function 'num-val :lambda-list '(m))
(cl:defmethod num-val ((m <leftrobot-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader real_robot_control-srv:num-val is deprecated.  Use real_robot_control-srv:num instead.")
  (num m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <leftrobot-request>) ostream)
  "Serializes a message object of type '<leftrobot-request>"
  (cl:let* ((signed (cl:slot-value msg 'num)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <leftrobot-request>) istream)
  "Deserializes a message object of type '<leftrobot-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<leftrobot-request>)))
  "Returns string type for a service object of type '<leftrobot-request>"
  "real_robot_control/leftrobotRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'leftrobot-request)))
  "Returns string type for a service object of type 'leftrobot-request"
  "real_robot_control/leftrobotRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<leftrobot-request>)))
  "Returns md5sum for a message object of type '<leftrobot-request>"
  "7b18929c222448a67fda0cd87f44304d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'leftrobot-request)))
  "Returns md5sum for a message object of type 'leftrobot-request"
  "7b18929c222448a67fda0cd87f44304d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<leftrobot-request>)))
  "Returns full string definition for message of type '<leftrobot-request>"
  (cl:format cl:nil "#目标值~%int32 num~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'leftrobot-request)))
  "Returns full string definition for message of type 'leftrobot-request"
  (cl:format cl:nil "#目标值~%int32 num~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <leftrobot-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <leftrobot-request>))
  "Converts a ROS message object to a list"
  (cl:list 'leftrobot-request
    (cl:cons ':num (num msg))
))
;//! \htmlinclude leftrobot-response.msg.html

(cl:defclass <leftrobot-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:integer
    :initform 0))
)

(cl:defclass leftrobot-response (<leftrobot-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <leftrobot-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'leftrobot-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name real_robot_control-srv:<leftrobot-response> is deprecated: use real_robot_control-srv:leftrobot-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <leftrobot-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader real_robot_control-srv:result-val is deprecated.  Use real_robot_control-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <leftrobot-response>) ostream)
  "Serializes a message object of type '<leftrobot-response>"
  (cl:let* ((signed (cl:slot-value msg 'result)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <leftrobot-response>) istream)
  "Deserializes a message object of type '<leftrobot-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<leftrobot-response>)))
  "Returns string type for a service object of type '<leftrobot-response>"
  "real_robot_control/leftrobotResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'leftrobot-response)))
  "Returns string type for a service object of type 'leftrobot-response"
  "real_robot_control/leftrobotResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<leftrobot-response>)))
  "Returns md5sum for a message object of type '<leftrobot-response>"
  "7b18929c222448a67fda0cd87f44304d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'leftrobot-response)))
  "Returns md5sum for a message object of type 'leftrobot-response"
  "7b18929c222448a67fda0cd87f44304d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<leftrobot-response>)))
  "Returns full string definition for message of type '<leftrobot-response>"
  (cl:format cl:nil "#最终结果~%int32 result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'leftrobot-response)))
  "Returns full string definition for message of type 'leftrobot-response"
  (cl:format cl:nil "#最终结果~%int32 result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <leftrobot-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <leftrobot-response>))
  "Converts a ROS message object to a list"
  (cl:list 'leftrobot-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'leftrobot)))
  'leftrobot-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'leftrobot)))
  'leftrobot-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'leftrobot)))
  "Returns string type for a service object of type '<leftrobot>"
  "real_robot_control/leftrobot")