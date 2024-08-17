; Auto-generated. Do not edit!


(cl:in-package real_robot_control-srv)


;//! \htmlinclude leftrobotsrv-request.msg.html

(cl:defclass <leftrobotsrv-request> (roslisp-msg-protocol:ros-message)
  ((num
    :reader num
    :initarg :num
    :type cl:integer
    :initform 0))
)

(cl:defclass leftrobotsrv-request (<leftrobotsrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <leftrobotsrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'leftrobotsrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name real_robot_control-srv:<leftrobotsrv-request> is deprecated: use real_robot_control-srv:leftrobotsrv-request instead.")))

(cl:ensure-generic-function 'num-val :lambda-list '(m))
(cl:defmethod num-val ((m <leftrobotsrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader real_robot_control-srv:num-val is deprecated.  Use real_robot_control-srv:num instead.")
  (num m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <leftrobotsrv-request>) ostream)
  "Serializes a message object of type '<leftrobotsrv-request>"
  (cl:let* ((signed (cl:slot-value msg 'num)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <leftrobotsrv-request>) istream)
  "Deserializes a message object of type '<leftrobotsrv-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<leftrobotsrv-request>)))
  "Returns string type for a service object of type '<leftrobotsrv-request>"
  "real_robot_control/leftrobotsrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'leftrobotsrv-request)))
  "Returns string type for a service object of type 'leftrobotsrv-request"
  "real_robot_control/leftrobotsrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<leftrobotsrv-request>)))
  "Returns md5sum for a message object of type '<leftrobotsrv-request>"
  "7b18929c222448a67fda0cd87f44304d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'leftrobotsrv-request)))
  "Returns md5sum for a message object of type 'leftrobotsrv-request"
  "7b18929c222448a67fda0cd87f44304d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<leftrobotsrv-request>)))
  "Returns full string definition for message of type '<leftrobotsrv-request>"
  (cl:format cl:nil "#目标值~%int32 num~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'leftrobotsrv-request)))
  "Returns full string definition for message of type 'leftrobotsrv-request"
  (cl:format cl:nil "#目标值~%int32 num~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <leftrobotsrv-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <leftrobotsrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'leftrobotsrv-request
    (cl:cons ':num (num msg))
))
;//! \htmlinclude leftrobotsrv-response.msg.html

(cl:defclass <leftrobotsrv-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:integer
    :initform 0))
)

(cl:defclass leftrobotsrv-response (<leftrobotsrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <leftrobotsrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'leftrobotsrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name real_robot_control-srv:<leftrobotsrv-response> is deprecated: use real_robot_control-srv:leftrobotsrv-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <leftrobotsrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader real_robot_control-srv:result-val is deprecated.  Use real_robot_control-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <leftrobotsrv-response>) ostream)
  "Serializes a message object of type '<leftrobotsrv-response>"
  (cl:let* ((signed (cl:slot-value msg 'result)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <leftrobotsrv-response>) istream)
  "Deserializes a message object of type '<leftrobotsrv-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<leftrobotsrv-response>)))
  "Returns string type for a service object of type '<leftrobotsrv-response>"
  "real_robot_control/leftrobotsrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'leftrobotsrv-response)))
  "Returns string type for a service object of type 'leftrobotsrv-response"
  "real_robot_control/leftrobotsrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<leftrobotsrv-response>)))
  "Returns md5sum for a message object of type '<leftrobotsrv-response>"
  "7b18929c222448a67fda0cd87f44304d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'leftrobotsrv-response)))
  "Returns md5sum for a message object of type 'leftrobotsrv-response"
  "7b18929c222448a67fda0cd87f44304d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<leftrobotsrv-response>)))
  "Returns full string definition for message of type '<leftrobotsrv-response>"
  (cl:format cl:nil "#最终结果~%int32 result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'leftrobotsrv-response)))
  "Returns full string definition for message of type 'leftrobotsrv-response"
  (cl:format cl:nil "#最终结果~%int32 result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <leftrobotsrv-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <leftrobotsrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'leftrobotsrv-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'leftrobotsrv)))
  'leftrobotsrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'leftrobotsrv)))
  'leftrobotsrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'leftrobotsrv)))
  "Returns string type for a service object of type '<leftrobotsrv>"
  "real_robot_control/leftrobotsrv")