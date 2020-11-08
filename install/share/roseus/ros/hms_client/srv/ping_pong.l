;; Auto-generated. Do not edit!


(when (boundp 'hms_client::ping_pong)
  (if (not (find-package "HMS_CLIENT"))
    (make-package "HMS_CLIENT"))
  (shadow 'ping_pong (find-package "HMS_CLIENT")))
(unless (find-package "HMS_CLIENT::PING_PONG")
  (make-package "HMS_CLIENT::PING_PONG"))
(unless (find-package "HMS_CLIENT::PING_PONGREQUEST")
  (make-package "HMS_CLIENT::PING_PONGREQUEST"))
(unless (find-package "HMS_CLIENT::PING_PONGRESPONSE")
  (make-package "HMS_CLIENT::PING_PONGRESPONSE"))

(in-package "ROS")





(defclass hms_client::ping_pongRequest
  :super ros::object
  :slots (_node_name ))

(defmethod hms_client::ping_pongRequest
  (:init
   (&key
    ((:node_name __node_name) "")
    )
   (send-super :init)
   (setq _node_name (string __node_name))
   self)
  (:node_name
   (&optional __node_name)
   (if __node_name (setq _node_name __node_name)) _node_name)
  (:serialization-length
   ()
   (+
    ;; string _node_name
    4 (length _node_name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _node_name
       (write-long (length _node_name) s) (princ _node_name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _node_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _node_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass hms_client::ping_pongResponse
  :super ros::object
  :slots (_msg _health _error_code ))

(defmethod hms_client::ping_pongResponse
  (:init
   (&key
    ((:msg __msg) (instance hms_client::hms_msg :init))
    ((:health __health) 0)
    ((:error_code __error_code) 0)
    )
   (send-super :init)
   (setq _msg __msg)
   (setq _health (round __health))
   (setq _error_code (round __error_code))
   self)
  (:msg
   (&rest __msg)
   (if (keywordp (car __msg))
       (send* _msg __msg)
     (progn
       (if __msg (setq _msg (car __msg)))
       _msg)))
  (:health
   (&optional __health)
   (if __health (setq _health __health)) _health)
  (:error_code
   (&optional __error_code)
   (if __error_code (setq _error_code __error_code)) _error_code)
  (:serialization-length
   ()
   (+
    ;; hms_client/hms_msg _msg
    (send _msg :serialization-length)
    ;; int64 _health
    8
    ;; int64 _error_code
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; hms_client/hms_msg _msg
       (send _msg :serialize s)
     ;; int64 _health
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _health (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _health) (= (length (_health . bv)) 2)) ;; bignum
              (write-long (ash (elt (_health . bv) 0) 0) s)
              (write-long (ash (elt (_health . bv) 1) -1) s))
             ((and (class _health) (= (length (_health . bv)) 1)) ;; big1
              (write-long (elt (_health . bv) 0) s)
              (write-long (if (>= _health 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _health s)(write-long (if (>= _health 0) 0 #xffffffff) s)))
     ;; int64 _error_code
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _error_code (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _error_code) (= (length (_error_code . bv)) 2)) ;; bignum
              (write-long (ash (elt (_error_code . bv) 0) 0) s)
              (write-long (ash (elt (_error_code . bv) 1) -1) s))
             ((and (class _error_code) (= (length (_error_code . bv)) 1)) ;; big1
              (write-long (elt (_error_code . bv) 0) s)
              (write-long (if (>= _error_code 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _error_code s)(write-long (if (>= _error_code 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; hms_client/hms_msg _msg
     (send _msg :deserialize buf ptr-) (incf ptr- (send _msg :serialization-length))
   ;; int64 _health
#+(or :alpha :irix6 :x86_64)
      (setf _health (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _health (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _error_code
#+(or :alpha :irix6 :x86_64)
      (setf _error_code (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _error_code (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass hms_client::ping_pong
  :super ros::object
  :slots ())

(setf (get hms_client::ping_pong :md5sum-) "f8a6e80cda99ecae7bfeb48f745404ba")
(setf (get hms_client::ping_pong :datatype-) "hms_client/ping_pong")
(setf (get hms_client::ping_pong :request) hms_client::ping_pongRequest)
(setf (get hms_client::ping_pong :response) hms_client::ping_pongResponse)

(defmethod hms_client::ping_pongRequest
  (:response () (instance hms_client::ping_pongResponse :init)))

(setf (get hms_client::ping_pongRequest :md5sum-) "f8a6e80cda99ecae7bfeb48f745404ba")
(setf (get hms_client::ping_pongRequest :datatype-) "hms_client/ping_pongRequest")
(setf (get hms_client::ping_pongRequest :definition-)
      "#request 
string node_name
---
#response 
hms_msg msg
int64 health
int64 error_code



================================================================================
MSG: hms_client/hms_msg
Header header
string temp1
float32 temp2
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

(setf (get hms_client::ping_pongResponse :md5sum-) "f8a6e80cda99ecae7bfeb48f745404ba")
(setf (get hms_client::ping_pongResponse :datatype-) "hms_client/ping_pongResponse")
(setf (get hms_client::ping_pongResponse :definition-)
      "#request 
string node_name
---
#response 
hms_msg msg
int64 health
int64 error_code



================================================================================
MSG: hms_client/hms_msg
Header header
string temp1
float32 temp2
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



(provide :hms_client/ping_pong "f8a6e80cda99ecae7bfeb48f745404ba")


