
(cl:in-package :asdf)

(defsystem "robot_planning-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "force_pub" :depends-on ("_package_force_pub"))
    (:file "_package_force_pub" :depends-on ("_package"))
  ))