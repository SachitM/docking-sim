; Auto-generated. Do not edit!


(cl:in-package hms_client-msg)


;//! \htmlinclude hms_msg.msg.html

(cl:defclass <hms_msg> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (temp1
    :reader temp1
    :initarg :temp1
    :type cl:string
    :initform "")
   (temp2
    :reader temp2
    :initarg :temp2
    :type cl:float
    :initform 0.0))
)

(cl:defclass hms_msg (<hms_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <hms_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'hms_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hms_client-msg:<hms_msg> is deprecated: use hms_client-msg:hms_msg instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <hms_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hms_client-msg:header-val is deprecated.  Use hms_client-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'temp1-val :lambda-list '(m))
(cl:defmethod temp1-val ((m <hms_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hms_client-msg:temp1-val is deprecated.  Use hms_client-msg:temp1 instead.")
  (temp1 m))

(cl:ensure-generic-function 'temp2-val :lambda-list '(m))
(cl:defmethod temp2-val ((m <hms_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hms_client-msg:temp2-val is deprecated.  Use hms_client-msg:temp2 instead.")
  (temp2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <hms_msg>) ostream)
  "Serializes a message object of type '<hms_msg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'temp1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'temp1))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'temp2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <hms_msg>) istream)
  "Deserializes a message object of type '<hms_msg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'temp1) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'temp1) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'temp2) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<hms_msg>)))
  "Returns string type for a message object of type '<hms_msg>"
  "hms_client/hms_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'hms_msg)))
  "Returns string type for a message object of type 'hms_msg"
  "hms_client/hms_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<hms_msg>)))
  "Returns md5sum for a message object of type '<hms_msg>"
  "64608a45b1add361d21d3cecf93db1f6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'hms_msg)))
  "Returns md5sum for a message object of type 'hms_msg"
  "64608a45b1add361d21d3cecf93db1f6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<hms_msg>)))
  "Returns full string definition for message of type '<hms_msg>"
  (cl:format cl:nil "Header header~%string temp1~%float32 temp2~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'hms_msg)))
  "Returns full string definition for message of type 'hms_msg"
  (cl:format cl:nil "Header header~%string temp1~%float32 temp2~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <hms_msg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'temp1))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <hms_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'hms_msg
    (cl:cons ':header (header msg))
    (cl:cons ':temp1 (temp1 msg))
    (cl:cons ':temp2 (temp2 msg))
))
