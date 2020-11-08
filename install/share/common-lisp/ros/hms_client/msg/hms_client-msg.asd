
(cl:in-package :asdf)

(defsystem "hms_client-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "hms_msg" :depends-on ("_package_hms_msg"))
    (:file "_package_hms_msg" :depends-on ("_package"))
  ))