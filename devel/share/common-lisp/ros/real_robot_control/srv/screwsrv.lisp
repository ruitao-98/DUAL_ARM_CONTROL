; Auto-generated. Do not edit!


(cl:in-package real_robot_control-srv)


;//! \htmlinclude screwsrv-request.msg.html

(cl:defclass <screwsrv-request> (roslisp-msg-protocol:ros-message)
  ((num
    :reader num
    :initarg :num
    :type cl:integer
    :initform 0))
)

(cl:defclass screwsrv-request (<screwsrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <screwsrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'screwsrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name real_robot_control-srv:<screwsrv-request> is deprecated: use real_robot_control-srv:screwsrv-request instead.")))

(cl:ensure-generic-function 'num-val :lambda-list '(m))
(cl:defmethod num-val ((m <screwsrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader real_robot_control-srv:num-val is deprecated.  Use real_robot_control-srv:num instead.")
  (num m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <screwsrv-request>) ostream)
  "Serializes a message object of type '<screwsrv-request>"
  (cl:let* ((signed (cl:slot-value msg 'num)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <screwsrv-request>) istream)
  "Deserializes a message object of type '<screwsrv-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<screwsrv-request>)))
  "Returns string type for a service object of type '<screwsrv-request>"
  "real_robot_control/screwsrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'screwsrv-request)))
  "Returns string type for a service object of type 'screwsrv-request"
  "real_robot_control/screwsrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<screwsrv-request>)))
  "Returns md5sum for a message object of type '<screwsrv-request>"
  "7b18929c222448a67fda0cd87f44304d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'screwsrv-request)))
  "Returns md5sum for a message object of type 'screwsrv-request"
  "7b18929c222448a67fda0cd87f44304d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<screwsrv-request>)))
  "Returns full string definition for message of type '<screwsrv-request>"
  (cl:format cl:nil "#目标值~%int32 num~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'screwsrv-request)))
  "Returns full string definition for message of type 'screwsrv-request"
  (cl:format cl:nil "#目标值~%int32 num~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <screwsrv-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <screwsrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'screwsrv-request
    (cl:cons ':num (num msg))
))
;//! \htmlinclude screwsrv-response.msg.html

(cl:defclass <screwsrv-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:integer
    :initform 0))
)

(cl:defclass screwsrv-response (<screwsrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <screwsrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'screwsrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name real_robot_control-srv:<screwsrv-response> is deprecated: use real_robot_control-srv:screwsrv-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <screwsrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader real_robot_control-srv:result-val is deprecated.  Use real_robot_control-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <screwsrv-response>) ostream)
  "Serializes a message object of type '<screwsrv-response>"
  (cl:let* ((signed (cl:slot-value msg 'result)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <screwsrv-response>) istream)
  "Deserializes a message object of type '<screwsrv-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<screwsrv-response>)))
  "Returns string type for a service object of type '<screwsrv-response>"
  "real_robot_control/screwsrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'screwsrv-response)))
  "Returns string type for a service object of type 'screwsrv-response"
  "real_robot_control/screwsrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<screwsrv-response>)))
  "Returns md5sum for a message object of type '<screwsrv-response>"
  "7b18929c222448a67fda0cd87f44304d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'screwsrv-response)))
  "Returns md5sum for a message object of type 'screwsrv-response"
  "7b18929c222448a67fda0cd87f44304d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<screwsrv-response>)))
  "Returns full string definition for message of type '<screwsrv-response>"
  (cl:format cl:nil "#最终结果~%int32 result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'screwsrv-response)))
  "Returns full string definition for message of type 'screwsrv-response"
  (cl:format cl:nil "#最终结果~%int32 result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <screwsrv-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <screwsrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'screwsrv-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'screwsrv)))
  'screwsrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'screwsrv)))
  'screwsrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'screwsrv)))
  "Returns string type for a service object of type '<screwsrv>"
  "real_robot_control/screwsrv")