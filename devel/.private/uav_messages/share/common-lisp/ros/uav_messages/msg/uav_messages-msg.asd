
(cl:in-package :asdf)

(defsystem "uav_messages-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "GroundClothPoints" :depends-on ("_package_GroundClothPoints"))
    (:file "_package_GroundClothPoints" :depends-on ("_package"))
    (:file "LeftRightRow" :depends-on ("_package_LeftRightRow"))
    (:file "_package_LeftRightRow" :depends-on ("_package"))
  ))