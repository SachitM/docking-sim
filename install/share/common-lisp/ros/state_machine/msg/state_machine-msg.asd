
(cl:in-package :asdf)

(defsystem "state_machine-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "StateIn" :depends-on ("_package_StateIn"))
    (:file "_package_StateIn" :depends-on ("_package"))
    (:file "StateOut" :depends-on ("_package_StateOut"))
    (:file "_package_StateOut" :depends-on ("_package"))
  ))