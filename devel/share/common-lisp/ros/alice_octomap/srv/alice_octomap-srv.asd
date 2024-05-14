
(cl:in-package :asdf)

(defsystem "alice_octomap-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "octomap" :depends-on ("_package_octomap"))
    (:file "_package_octomap" :depends-on ("_package"))
  ))