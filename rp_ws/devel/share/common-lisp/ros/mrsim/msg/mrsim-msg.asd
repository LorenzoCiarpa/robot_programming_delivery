
(cl:in-package :asdf)

(defsystem "mrsim-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "rodom" :depends-on ("_package_rodom"))
    (:file "_package_rodom" :depends-on ("_package"))
  ))