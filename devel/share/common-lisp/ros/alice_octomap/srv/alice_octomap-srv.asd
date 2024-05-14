
(cl:in-package :asdf)

(defsystem "alice_octomap-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "octomap_srv_client" :depends-on ("_package_octomap_srv_client"))
    (:file "_package_octomap_srv_client" :depends-on ("_package"))
  ))