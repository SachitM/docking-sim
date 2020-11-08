; Auto-generated. Do not edit!


(cl:in-package hms_client-srv)


;//! \htmlinclude ping_pong-request.msg.html

(cl:defclass <ping_pong-request> (roslisp-msg-protocol:ros-message)
  ((node_name
    :reader node_name
    :initarg :node_name
    :type cl:string
    :initform ""))
)

(cl:defclass ping_pong-request (<ping_pong-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ping_pong-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ping_pong-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hms_client-srv:<ping_pong-request> is deprecated: use hms_client-srv:ping_pong-request instead.")))

(cl:ensure-generic-function 'node_name-val :lambda-list '(m))
(cl:defmethod node_name-val ((m <ping_pong-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hms_client-srv:node_name-val is deprecated.  Use hms_client-srv:node_name instead.")
  (node_name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ping_pong-request>) ostream)
  "Serializes a message object of type '<ping_pong-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'node_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'node_name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ping_pong-request>) istream)
  "Deserializes a message object of type '<ping_pong-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'node_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'node_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ping_pong-request>)))
  "Returns string type for a service object of type '<ping_pong-request>"
  "hms_client/ping_pongRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ping_pong-request)))
  "Returns string type for a service object of type 'ping_pong-request"
  "hms_client/ping_pongRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ping_pong-request>)))
  "Returns md5sum for a message object of type '<ping_pong-request>"
  "f8a6e80cda99ecae7bfeb48f745404ba")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ping_pong-request)))
  "Returns md5sum for a message object of type 'ping_pong-request"
  "f8a6e80cda99ecae7bfeb48f745404ba")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ping_pong-request>)))
  "Returns full string definition for message of type '<ping_pong-request>"
  (cl:format cl:nil "#request ~%string node_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ping_pong-request)))
  "Returns full string definition for message of type 'ping_pong-request"
  (cl:format cl:nil "#request ~%string node_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ping_pong-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'node_name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ping_pong-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ping_pong-request
    (cl:cons ':node_name (node_name msg))
))
;//! \htmlinclude ping_pong-response.msg.html

(cl:defclass <ping_pong-response> (roslisp-msg-protocol:ros-message)
  ((msg
    :reader msg
    :initarg :msg
    :type hms_client-msg:hms_msg
    :initform (cl:make-instance 'hms_client-msg:hms_msg))
   (health
    :reader health
    :initarg :health
    :type cl:integer
    :initform 0)
   (error_code
    :reader error_code
    :initarg :error_code
    :type cl:integer
    :initform 0))
)

(cl:defclass ping_pong-response (<ping_pong-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ping_pong-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ping_pong-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hms_client-srv:<ping_pong-response> is deprecated: use hms_client-srv:ping_pong-response instead.")))

(cl:ensure-generic-function 'msg-val :lambda-list '(m))
(cl:defmethod msg-val ((m <ping_pong-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hms_client-srv:msg-val is deprecated.  Use hms_client-srv:msg instead.")
  (msg m))

(cl:ensure-generic-function 'health-val :lambda-list '(m))
(cl:defmethod health-val ((m <ping_pong-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hms_client-srv:health-val is deprecated.  Use hms_client-srv:health instead.")
  (health m))

(cl:ensure-generic-function 'error_code-val :lambda-list '(m))
(cl:defmethod error_code-val ((m <ping_pong-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hms_client-srv:error_code-val is deprecated.  Use hms_client-srv:error_code instead.")
  (error_code m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ping_pong-response>) ostream)
  "Serializes a message object of type '<ping_pong-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'msg) ostream)
  (cl:let* ((signed (cl:slot-value msg 'health)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'error_code)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ping_pong-response>) istream)
  "Deserializes a message object of type '<ping_pong-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'msg) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'health) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'error_code) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ping_pong-response>)))
  "Returns string type for a service object of type '<ping_pong-response>"
  "hms_client/ping_pongResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ping_pong-response)))
  "Returns string type for a service object of type 'ping_pong-response"
  "hms_client/ping_pongResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ping_pong-response>)))
  "Returns md5sum for a message object of type '<ping_pong-response>"
  "f8a6e80cda99ecae7bfeb48f745404ba")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ping_pong-response)))
  "Returns md5sum for a message object of type 'ping_pong-response"
  "f8a6e80cda99ecae7bfeb48f745404ba")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ping_pong-response>)))
  "Returns full string definition for message of type '<ping_pong-response>"
  (cl:format cl:nil "#response ~%hms_msg msg~%int64 health~%int64 error_code~%~%~%~%================================================================================~%MSG: hms_client/hms_msg~%Header header~%string temp1~%float32 temp2~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ping_pong-response)))
  "Returns full string definition for message of type 'ping_pong-response"
  (cl:format cl:nil "#response ~%hms_msg msg~%int64 health~%int64 error_code~%~%~%~%================================================================================~%MSG: hms_client/hms_msg~%Header header~%string temp1~%float32 temp2~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ping_pong-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'msg))
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ping_pong-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ping_pong-response
    (cl:cons ':msg (msg msg))
    (cl:cons ':health (health msg))
    (cl:cons ':error_code (error_code msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ping_pong)))
  'ping_pong-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ping_pong)))
  'ping_pong-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ping_pong)))
  "Returns string type for a service object of type '<ping_pong>"
  "hms_client/ping_pong")