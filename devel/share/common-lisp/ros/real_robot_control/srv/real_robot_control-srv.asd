
(cl:in-package :asdf)

(defsystem "real_robot_control-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "screwsrv" :depends-on ("_package_screwsrv"))
    (:file "_package_screwsrv" :depends-on ("_package"))
  ))