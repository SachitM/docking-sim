; Auto-generated. Do not edit!


(cl:in-package state_machine-msg)


;//! \htmlinclude StateIn.msg.html

(cl:defclass <StateIn> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (TransState
    :reader TransState
    :initarg :TransState
    :type cl:fixnum
    :initform 0)
   (StateTransitionCond
    :reader StateTransitionCond
    :initarg :StateTransitionCond
    :type cl:fixnum
    :initform 0))
)

(cl:defclass StateIn (<StateIn>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StateIn>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StateIn)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name state_machine-msg:<StateIn> is deprecated: use state_machine-msg:StateIn instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <StateIn>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_machine-msg:header-val is deprecated.  Use state_machine-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'TransState-val :lambda-list '(m))
(cl:defmethod TransState-val ((m <StateIn>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_machine-msg:TransState-val is deprecated.  Use state_machine-msg:TransState instead.")
  (TransState m))

(cl:ensure-generic-function 'StateTransitionCond-val :lambda-list '(m))
(cl:defmethod StateTransitionCond-val ((m <StateIn>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_machine-msg:StateTransitionCond-val is deprecated.  Use state_machine-msg:StateTransitionCond instead.")
  (StateTransitionCond m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StateIn>) ostream)
  "Serializes a message object of type '<StateIn>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'TransState)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'StateTransitionCond)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StateIn>) istream)
  "Deserializes a message object of type '<StateIn>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'TransState)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'StateTransitionCond)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StateIn>)))
  "Returns string type for a message object of type '<StateIn>"
  "state_machine/StateIn")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StateIn)))
  "Returns string type for a message object of type 'StateIn"
  "state_machine/StateIn")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StateIn>)))
  "Returns md5sum for a message object of type '<StateIn>"
  "1c6322d5ada42d15cd42840808483749")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StateIn)))
  "Returns md5sum for a message object of type 'StateIn"
  "1c6322d5ada42d15cd42840808483749")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StateIn>)))
  "Returns full string definition for message of type '<StateIn>"
  (cl:format cl:nil "Header header ~%uint8 TransState~%uint8 StateTransitionCond~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StateIn)))
  "Returns full string definition for message of type 'StateIn"
  (cl:format cl:nil "Header header ~%uint8 TransState~%uint8 StateTransitionCond~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StateIn>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StateIn>))
  "Converts a ROS message object to a list"
  (cl:list 'StateIn
    (cl:cons ':header (header msg))
    (cl:cons ':TransState (TransState msg))
    (cl:cons ':StateTransitionCond (StateTransitionCond msg))
))
