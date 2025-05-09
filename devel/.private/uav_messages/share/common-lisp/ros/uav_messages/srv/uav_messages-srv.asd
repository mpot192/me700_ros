
(cl:in-package :asdf)

(defsystem "uav_messages-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :trajectory_msgs-msg
)
  :components ((:file "_package")
    (:file "EsdfQuery" :depends-on ("_package_EsdfQuery"))
    (:file "_package_EsdfQuery" :depends-on ("_package"))
    (:file "Heartbeat" :depends-on ("_package_Heartbeat"))
    (:file "_package_Heartbeat" :depends-on ("_package"))
    (:file "Height" :depends-on ("_package_Height"))
    (:file "_package_Height" :depends-on ("_package"))
    (:file "TrajectoryFollowerStatus" :depends-on ("_package_TrajectoryFollowerStatus"))
    (:file "_package_TrajectoryFollowerStatus" :depends-on ("_package"))
    (:file "TrajectoryTarget" :depends-on ("_package_TrajectoryTarget"))
    (:file "_package_TrajectoryTarget" :depends-on ("_package"))
    (:file "TrajectoryTargetTest" :depends-on ("_package_TrajectoryTargetTest"))
    (:file "_package_TrajectoryTargetTest" :depends-on ("_package"))
  ))