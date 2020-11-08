
(cl:in-package :asdf)

(defsystem "hms_client-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :hms_client-msg
)
  :components ((:file "_package")
    (:file "ping_pong" :depends-on ("_package_ping_pong"))
    (:file "_package_ping_pong" :depends-on ("_package"))
  ))