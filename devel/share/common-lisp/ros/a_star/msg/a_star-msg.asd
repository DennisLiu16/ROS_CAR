
(cl:in-package :asdf)

(defsystem "a_star-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "isReached" :depends-on ("_package_isReached"))
    (:file "_package_isReached" :depends-on ("_package"))
  ))