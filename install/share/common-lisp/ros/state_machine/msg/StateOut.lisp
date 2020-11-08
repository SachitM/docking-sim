; Auto-generated. Do not edit!


(cl:in-package state_machine-msg)


;//! \htmlinclude StateOut.msg.html

(cl:defclass <StateOut> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (Diagnostics
    :reader Diagnostics
    :initarg :Diagnostics
    :type cl:string
    :initform "")
   (PodInfo
    :reader PodInfo
    :initarg :PodInfo
    :type cl:fixnum
    :initform 0)
   (HMSCheck
    :reader HMSCheck
    :initarg :HMSCheck
    :type cl:fixnum
    :initform 0)
   (OperationMode
    :reader OperationMode
    :initarg :OperationMode
    :type cl:fixnum
    :initform 0)
   (PrevState
    :reader PrevState
    :initarg :PrevState
    :type cl:fixnum
    :initform 0)
   (CurrState
    :reader CurrState
    :initarg :CurrState
    :type cl:fixnum
    :initform 0))
)

(cl:defclass StateOut (<StateOut>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StateOut>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StateOut)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name state_machine-msg:<StateOut> is deprecated: use state_machine-msg:StateOut instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <StateOut>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_machine-msg:header-val is deprecated.  Use state_machine-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'Diagnostics-val :lambda-list '(m))
(cl:defmethod Diagnostics-val ((m <StateOut>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_machine-msg:Diagnostics-val is deprecated.  Use state_machine-msg:Diagnostics instead.")
  (Diagnostics m))

(cl:ensure-generic-function 'PodInfo-val :lambda-list '(m))
(cl:defmethod PodInfo-val ((m <StateOut>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_machine-msg:PodInfo-val is deprecated.  Use state_machine-msg:PodInfo instead.")
  (PodInfo m))

(cl:ensure-generic-function 'HMSCheck-val :lambda-list '(m))
(cl:defmethod HMSCheck-val ((m <StateOut>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_machine-msg:HMSCheck-val is deprecated.  Use state_machine-msg:HMSCheck instead.")
  (HMSCheck m))

(cl:ensure-generic-function 'OperationMode-val :lambda-list '(m))
(cl:defmethod OperationMode-val ((m <StateOut>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_machine-msg:OperationMode-val is deprecated.  Use state_machine-msg:OperationMode instead.")
  (OperationMode m))

(cl:ensure-generic-function 'PrevState-val :lambda-list '(m))
(cl:defmethod PrevState-val ((m <StateOut>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_machine-msg:PrevState-val is deprecated.  Use state_machine-msg:PrevState instead.")
  (PrevState m))

(cl:ensure-generic-function 'CurrState-val :lambda-list '(m))
(cl:defmethod CurrState-val ((m <StateOut>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_machine-msg:CurrState-val is deprecated.  Use state_machine-msg:CurrState instead.")
  (CurrState m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<StateOut>)))
    "Constants for message type '<StateOut>"
  '((:STATE_IDLE . 0)
    (:STATE_P2P . 1)
    (:STATE_IDENTIFY . 2)
    (:STATE_APPROACH . 3)
    (:STATE_VERIFY . 4)
    (:STATE_RETRACE . 5)
    (:STATE_LOCK . 6)
    (:STATE_UNLOCK . 7)
    (:STATE_EHS . 8)
    (:OPERATIONMODE_PICKUP . 1)
    (:OPERATIONMODE_DROPOFF . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'StateOut)))
    "Constants for message type 'StateOut"
  '((:STATE_IDLE . 0)
    (:STATE_P2P . 1)
    (:STATE_IDENTIFY . 2)
    (:STATE_APPROACH . 3)
    (:STATE_VERIFY . 4)
    (:STATE_RETRACE . 5)
    (:STATE_LOCK . 6)
    (:STATE_UNLOCK . 7)
    (:STATE_EHS . 8)
    (:OPERATIONMODE_PICKUP . 1)
    (:OPERATIONMODE_DROPOFF . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StateOut>) ostream)
  "Serializes a message object of type '<StateOut>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'Diagnostics))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'Diagnostics))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'PodInfo)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'HMSCheck)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'OperationMode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'PrevState)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'CurrState)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StateOut>) istream)
  "Deserializes a message object of type '<StateOut>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Diagnostics) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'Diagnostics) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'PodInfo)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'HMSCheck)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'OperationMode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'PrevState)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'CurrState)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StateOut>)))
  "Returns string type for a message object of type '<StateOut>"
  "state_machine/StateOut")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StateOut)))
  "Returns string type for a message object of type 'StateOut"
  "state_machine/StateOut")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StateOut>)))
  "Returns md5sum for a message object of type '<StateOut>"
  "709ee385b8a9e470a0649c6afc4d01c8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StateOut)))
  "Returns md5sum for a message object of type 'StateOut"
  "709ee385b8a9e470a0649c6afc4d01c8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StateOut>)))
  "Returns full string definition for message of type '<StateOut>"
  (cl:format cl:nil "uint8 State_Idle=0~%uint8 State_P2P=1~%uint8 State_Identify=2~%uint8 State_Approach=3~%uint8 State_Verify=4~%uint8 State_Retrace=5~%uint8 State_Lock=6~%uint8 State_Unlock=7~%uint8 State_EHS= 8~%uint8 OperationMode_Pickup = 1~%uint8 OperationMode_DropOff = 2~%Header header ~%string Diagnostics~%uint8 PodInfo~%uint8 HMSCheck~%uint8 OperationMode~%uint8 PrevState~%uint8 CurrState~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StateOut)))
  "Returns full string definition for message of type 'StateOut"
  (cl:format cl:nil "uint8 State_Idle=0~%uint8 State_P2P=1~%uint8 State_Identify=2~%uint8 State_Approach=3~%uint8 State_Verify=4~%uint8 State_Retrace=5~%uint8 State_Lock=6~%uint8 State_Unlock=7~%uint8 State_EHS= 8~%uint8 OperationMode_Pickup = 1~%uint8 OperationMode_DropOff = 2~%Header header ~%string Diagnostics~%uint8 PodInfo~%uint8 HMSCheck~%uint8 OperationMode~%uint8 PrevState~%uint8 CurrState~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StateOut>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'Diagnostics))
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StateOut>))
  "Converts a ROS message object to a list"
  (cl:list 'StateOut
    (cl:cons ':header (header msg))
    (cl:cons ':Diagnostics (Diagnostics msg))
    (cl:cons ':PodInfo (PodInfo msg))
    (cl:cons ':HMSCheck (HMSCheck msg))
    (cl:cons ':OperationMode (OperationMode msg))
    (cl:cons ':PrevState (PrevState msg))
    (cl:cons ':CurrState (CurrState msg))
))
