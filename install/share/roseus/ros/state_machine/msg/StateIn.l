;; Auto-generated. Do not edit!


(when (boundp 'state_machine::StateIn)
  (if (not (find-package "STATE_MACHINE"))
    (make-package "STATE_MACHINE"))
  (shadow 'StateIn (find-package "STATE_MACHINE")))
(unless (find-package "STATE_MACHINE::STATEIN")
  (make-package "STATE_MACHINE::STATEIN"))

(in-package "ROS")
;;//! \htmlinclude StateIn.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass state_machine::StateIn
  :super ros::object
  :slots (_header _TransState _StateTransitionCond ))

(defmethod state_machine::StateIn
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:TransState __TransState) 0)
    ((:StateTransitionCond __StateTransitionCond) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _TransState (round __TransState))
   (setq _StateTransitionCond (round __StateTransitionCond))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:TransState
   (&optional __TransState)
   (if __TransState (setq _TransState __TransState)) _TransState)
  (:StateTransitionCond
   (&optional __StateTransitionCond)
   (if __StateTransitionCond (setq _StateTransitionCond __StateTransitionCond)) _StateTransitionCond)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _TransState
    1
    ;; uint8 _StateTransitionCond
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _TransState
       (write-byte _TransState s)
     ;; uint8 _StateTransitionCond
       (write-byte _StateTransitionCond s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _TransState
     (setq _TransState (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _StateTransitionCond
     (setq _StateTransitionCond (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get state_machine::StateIn :md5sum-) "1c6322d5ada42d15cd42840808483749")
(setf (get state_machine::StateIn :datatype-) "state_machine/StateIn")
(setf (get state_machine::StateIn :definition-)
      "Header header 
uint8 TransState
uint8 StateTransitionCond

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :state_machine/StateIn "1c6322d5ada42d15cd42840808483749")


