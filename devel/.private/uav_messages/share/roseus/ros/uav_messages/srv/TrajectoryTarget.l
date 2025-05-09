;; Auto-generated. Do not edit!


(when (boundp 'uav_messages::TrajectoryTarget)
  (if (not (find-package "UAV_MESSAGES"))
    (make-package "UAV_MESSAGES"))
  (shadow 'TrajectoryTarget (find-package "UAV_MESSAGES")))
(unless (find-package "UAV_MESSAGES::TRAJECTORYTARGET")
  (make-package "UAV_MESSAGES::TRAJECTORYTARGET"))
(unless (find-package "UAV_MESSAGES::TRAJECTORYTARGETREQUEST")
  (make-package "UAV_MESSAGES::TRAJECTORYTARGETREQUEST"))
(unless (find-package "UAV_MESSAGES::TRAJECTORYTARGETRESPONSE")
  (make-package "UAV_MESSAGES::TRAJECTORYTARGETRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(intern "*OK*" (find-package "UAV_MESSAGES::TRAJECTORYTARGETREQUEST"))
(shadow '*OK* (find-package "UAV_MESSAGES::TRAJECTORYTARGETREQUEST"))
(defconstant uav_messages::TrajectoryTargetRequest::*OK* 0)
(intern "*ERR_NO_PATH_FOUND*" (find-package "UAV_MESSAGES::TRAJECTORYTARGETREQUEST"))
(shadow '*ERR_NO_PATH_FOUND* (find-package "UAV_MESSAGES::TRAJECTORYTARGETREQUEST"))
(defconstant uav_messages::TrajectoryTargetRequest::*ERR_NO_PATH_FOUND* 1)

(defun uav_messages::TrajectoryTargetRequest-to-symbol (const)
  (cond
        ((= const 0) 'uav_messages::TrajectoryTargetRequest::*OK*)
        ((= const 1) 'uav_messages::TrajectoryTargetRequest::*ERR_NO_PATH_FOUND*)
        (t nil)))

(defclass uav_messages::TrajectoryTargetRequest
  :super ros::object
  :slots (_waypoints _start_velocity _end_velocity _target_velocity ))

(defmethod uav_messages::TrajectoryTargetRequest
  (:init
   (&key
    ((:waypoints __waypoints) ())
    ((:start_velocity __start_velocity) (instance geometry_msgs::Vector3 :init))
    ((:end_velocity __end_velocity) (instance geometry_msgs::Vector3 :init))
    ((:target_velocity __target_velocity) (instance geometry_msgs::Vector3 :init))
    )
   (send-super :init)
   (setq _waypoints __waypoints)
   (setq _start_velocity __start_velocity)
   (setq _end_velocity __end_velocity)
   (setq _target_velocity __target_velocity)
   self)
  (:waypoints
   (&rest __waypoints)
   (if (keywordp (car __waypoints))
       (send* _waypoints __waypoints)
     (progn
       (if __waypoints (setq _waypoints (car __waypoints)))
       _waypoints)))
  (:start_velocity
   (&rest __start_velocity)
   (if (keywordp (car __start_velocity))
       (send* _start_velocity __start_velocity)
     (progn
       (if __start_velocity (setq _start_velocity (car __start_velocity)))
       _start_velocity)))
  (:end_velocity
   (&rest __end_velocity)
   (if (keywordp (car __end_velocity))
       (send* _end_velocity __end_velocity)
     (progn
       (if __end_velocity (setq _end_velocity (car __end_velocity)))
       _end_velocity)))
  (:target_velocity
   (&rest __target_velocity)
   (if (keywordp (car __target_velocity))
       (send* _target_velocity __target_velocity)
     (progn
       (if __target_velocity (setq _target_velocity (car __target_velocity)))
       _target_velocity)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point[] _waypoints
    (apply #'+ (send-all _waypoints :serialization-length)) 4
    ;; geometry_msgs/Vector3 _start_velocity
    (send _start_velocity :serialization-length)
    ;; geometry_msgs/Vector3 _end_velocity
    (send _end_velocity :serialization-length)
    ;; geometry_msgs/Vector3 _target_velocity
    (send _target_velocity :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point[] _waypoints
     (write-long (length _waypoints) s)
     (dolist (elem _waypoints)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Vector3 _start_velocity
       (send _start_velocity :serialize s)
     ;; geometry_msgs/Vector3 _end_velocity
       (send _end_velocity :serialize s)
     ;; geometry_msgs/Vector3 _target_velocity
       (send _target_velocity :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point[] _waypoints
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _waypoints (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _waypoints)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Vector3 _start_velocity
     (send _start_velocity :deserialize buf ptr-) (incf ptr- (send _start_velocity :serialization-length))
   ;; geometry_msgs/Vector3 _end_velocity
     (send _end_velocity :deserialize buf ptr-) (incf ptr- (send _end_velocity :serialization-length))
   ;; geometry_msgs/Vector3 _target_velocity
     (send _target_velocity :deserialize buf ptr-) (incf ptr- (send _target_velocity :serialization-length))
   ;;
   self)
  )

(defclass uav_messages::TrajectoryTargetResponse
  :super ros::object
  :slots (_success _error ))

(defmethod uav_messages::TrajectoryTargetResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:error __error) 0)
    )
   (send-super :init)
   (setq _success __success)
   (setq _error (round __error))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:error
   (&optional __error)
   (if __error (setq _error __error)) _error)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; uint8 _error
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; uint8 _error
       (write-byte _error s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8 _error
     (setq _error (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass uav_messages::TrajectoryTarget
  :super ros::object
  :slots ())

(setf (get uav_messages::TrajectoryTarget :md5sum-) "ccd1ece0eed8a932ade7a0f997a2a505")
(setf (get uav_messages::TrajectoryTarget :datatype-) "uav_messages/TrajectoryTarget")
(setf (get uav_messages::TrajectoryTarget :request) uav_messages::TrajectoryTargetRequest)
(setf (get uav_messages::TrajectoryTarget :response) uav_messages::TrajectoryTargetResponse)

(defmethod uav_messages::TrajectoryTargetRequest
  (:response () (instance uav_messages::TrajectoryTargetResponse :init)))

(setf (get uav_messages::TrajectoryTargetRequest :md5sum-) "ccd1ece0eed8a932ade7a0f997a2a505")
(setf (get uav_messages::TrajectoryTargetRequest :datatype-) "uav_messages/TrajectoryTargetRequest")
(setf (get uav_messages::TrajectoryTargetRequest :definition-)
      "uint8 OK=0
uint8 ERR_NO_PATH_FOUND=1

geometry_msgs/Point[] waypoints
geometry_msgs/Vector3 start_velocity
geometry_msgs/Vector3 end_velocity
geometry_msgs/Vector3 target_velocity

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
---
bool success
uint8 error
")

(setf (get uav_messages::TrajectoryTargetResponse :md5sum-) "ccd1ece0eed8a932ade7a0f997a2a505")
(setf (get uav_messages::TrajectoryTargetResponse :datatype-) "uav_messages/TrajectoryTargetResponse")
(setf (get uav_messages::TrajectoryTargetResponse :definition-)
      "uint8 OK=0
uint8 ERR_NO_PATH_FOUND=1

geometry_msgs/Point[] waypoints
geometry_msgs/Vector3 start_velocity
geometry_msgs/Vector3 end_velocity
geometry_msgs/Vector3 target_velocity

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
---
bool success
uint8 error
")



(provide :uav_messages/TrajectoryTarget "ccd1ece0eed8a932ade7a0f997a2a505")


