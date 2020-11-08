;; Auto-generated. Do not edit!


(when (boundp 'state_machine::StateOut)
  (if (not (find-package "STATE_MACHINE"))
    (make-package "STATE_MACHINE"))
  (shadow 'StateOut (find-package "STATE_MACHINE")))
(unless (find-package "STATE_MACHINE::STATEOUT")
  (make-package "STATE_MACHINE::STATEOUT"))

(in-package "ROS")
;;//! \htmlinclude StateOut.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*STATE_IDLE*" (find-package "STATE_MACHINE::STATEOUT"))
(shadow '*STATE_IDLE* (find-package "STATE_MACHINE::STATEOUT"))
(defconstant state_machine::StateOut::*STATE_IDLE* 0)
(intern "*STATE_P2P*" (find-package "STATE_MACHINE::STATEOUT"))
(shadow '*STATE_P2P* (find-package "STATE_MACHINE::STATEOUT"))
(defconstant state_machine::StateOut::*STATE_P2P* 1)
(intern "*STATE_IDENTIFY*" (find-package "STATE_MACHINE::STATEOUT"))
(shadow '*STATE_IDENTIFY* (find-package "STATE_MACHINE::STATEOUT"))
(defconstant state_machine::StateOut::*STATE_IDENTIFY* 2)
(intern "*STATE_APPROACH*" (find-package "STATE_MACHINE::STATEOUT"))
(shadow '*STATE_APPROACH* (find-package "STATE_MACHINE::STATEOUT"))
(defconstant state_machine::StateOut::*STATE_APPROACH* 3)
(intern "*STATE_VERIFY*" (find-package "STATE_MACHINE::STATEOUT"))
(shadow '*STATE_VERIFY* (find-package "STATE_MACHINE::STATEOUT"))
(defconstant state_machine::StateOut::*STATE_VERIFY* 4)
(intern "*STATE_RETRACE*" (find-package "STATE_MACHINE::STATEOUT"))
(shadow '*STATE_RETRACE* (find-package "STATE_MACHINE::STATEOUT"))
(defconstant state_machine::StateOut::*STATE_RETRACE* 5)
(intern "*STATE_LOCK*" (find-package "STATE_MACHINE::STATEOUT"))
(shadow '*STATE_LOCK* (find-package "STATE_MACHINE::STATEOUT"))
(defconstant state_machine::StateOut::*STATE_LOCK* 6)
(intern "*STATE_UNLOCK*" (find-package "STATE_MACHINE::STATEOUT"))
(shadow '*STATE_UNLOCK* (find-package "STATE_MACHINE::STATEOUT"))
(defconstant state_machine::StateOut::*STATE_UNLOCK* 7)
(intern "*STATE_EHS*" (find-package "STATE_MACHINE::STATEOUT"))
(shadow '*STATE_EHS* (find-package "STATE_MACHINE::STATEOUT"))
(defconstant state_machine::StateOut::*STATE_EHS* 8)
(intern "*OPERATIONMODE_PICKUP*" (find-package "STATE_MACHINE::STATEOUT"))
(shadow '*OPERATIONMODE_PICKUP* (find-package "STATE_MACHINE::STATEOUT"))
(defconstant state_machine::StateOut::*OPERATIONMODE_PICKUP* 1)
(intern "*OPERATIONMODE_DROPOFF*" (find-package "STATE_MACHINE::STATEOUT"))
(shadow '*OPERATIONMODE_DROPOFF* (find-package "STATE_MACHINE::STATEOUT"))
(defconstant state_machine::StateOut::*OPERATIONMODE_DROPOFF* 2)
(defclass state_machine::StateOut
  :super ros::object
  :slots (_header _Diagnostics _PodInfo _HMSCheck _OperationMode _PrevState _CurrState ))

(defmethod state_machine::StateOut
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:Diagnostics __Diagnostics) "")
    ((:PodInfo __PodInfo) 0)
    ((:HMSCheck __HMSCheck) 0)
    ((:OperationMode __OperationMode) 0)
    ((:PrevState __PrevState) 0)
    ((:CurrState __CurrState) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _Diagnostics (string __Diagnostics))
   (setq _PodInfo (round __PodInfo))
   (setq _HMSCheck (round __HMSCheck))
   (setq _OperationMode (round __OperationMode))
   (setq _PrevState (round __PrevState))
   (setq _CurrState (round __CurrState))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:Diagnostics
   (&optional __Diagnostics)
   (if __Diagnostics (setq _Diagnostics __Diagnostics)) _Diagnostics)
  (:PodInfo
   (&optional __PodInfo)
   (if __PodInfo (setq _PodInfo __PodInfo)) _PodInfo)
  (:HMSCheck
   (&optional __HMSCheck)
   (if __HMSCheck (setq _HMSCheck __HMSCheck)) _HMSCheck)
  (:OperationMode
   (&optional __OperationMode)
   (if __OperationMode (setq _OperationMode __OperationMode)) _OperationMode)
  (:PrevState
   (&optional __PrevState)
   (if __PrevState (setq _PrevState __PrevState)) _PrevState)
  (:CurrState
   (&optional __CurrState)
   (if __CurrState (setq _CurrState __CurrState)) _CurrState)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _Diagnostics
    4 (length _Diagnostics)
    ;; uint8 _PodInfo
    1
    ;; uint8 _HMSCheck
    1
    ;; uint8 _OperationMode
    1
    ;; uint8 _PrevState
    1
    ;; uint8 _CurrState
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _Diagnostics
       (write-long (length _Diagnostics) s) (princ _Diagnostics s)
     ;; uint8 _PodInfo
       (write-byte _PodInfo s)
     ;; uint8 _HMSCheck
       (write-byte _HMSCheck s)
     ;; uint8 _OperationMode
       (write-byte _OperationMode s)
     ;; uint8 _PrevState
       (write-byte _PrevState s)
     ;; uint8 _CurrState
       (write-byte _CurrState s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _Diagnostics
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _Diagnostics (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint8 _PodInfo
     (setq _PodInfo (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _HMSCheck
     (setq _HMSCheck (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _OperationMode
     (setq _OperationMode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _PrevState
     (setq _PrevState (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _CurrState
     (setq _CurrState (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get state_machine::StateOut :md5sum-) "709ee385b8a9e470a0649c6afc4d01c8")
(setf (get state_machine::StateOut :datatype-) "state_machine/StateOut")
(setf (get state_machine::StateOut :definition-)
      "uint8 State_Idle=0
uint8 State_P2P=1
uint8 State_Identify=2
uint8 State_Approach=3
uint8 State_Verify=4
uint8 State_Retrace=5
uint8 State_Lock=6
uint8 State_Unlock=7
uint8 State_EHS= 8
uint8 OperationMode_Pickup = 1
uint8 OperationMode_DropOff = 2
Header header 
string Diagnostics
uint8 PodInfo
uint8 HMSCheck
uint8 OperationMode
uint8 PrevState
uint8 CurrState
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



(provide :state_machine/StateOut "709ee385b8a9e470a0649c6afc4d01c8")


