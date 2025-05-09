;; Auto-generated. Do not edit!


(when (boundp 'uav_messages::TrajectoryTargetTest)
  (if (not (find-package "UAV_MESSAGES"))
    (make-package "UAV_MESSAGES"))
  (shadow 'TrajectoryTargetTest (find-package "UAV_MESSAGES")))
(unless (find-package "UAV_MESSAGES::TRAJECTORYTARGETTEST")
  (make-package "UAV_MESSAGES::TRAJECTORYTARGETTEST"))
(unless (find-package "UAV_MESSAGES::TRAJECTORYTARGETTESTREQUEST")
  (make-package "UAV_MESSAGES::TRAJECTORYTARGETTESTREQUEST"))
(unless (find-package "UAV_MESSAGES::TRAJECTORYTARGETTESTRESPONSE")
  (make-package "UAV_MESSAGES::TRAJECTORYTARGETTESTRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(if (not (find-package "TRAJECTORY_MSGS"))
  (ros::roseus-add-msgs "trajectory_msgs"))


(intern "*OK*" (find-package "UAV_MESSAGES::TRAJECTORYTARGETTESTREQUEST"))
(shadow '*OK* (find-package "UAV_MESSAGES::TRAJECTORYTARGETTESTREQUEST"))
(defconstant uav_messages::TrajectoryTargetTestRequest::*OK* 0)
(intern "*ERR_NO_PATH_FOUND*" (find-package "UAV_MESSAGES::TRAJECTORYTARGETTESTREQUEST"))
(shadow '*ERR_NO_PATH_FOUND* (find-package "UAV_MESSAGES::TRAJECTORYTARGETTESTREQUEST"))
(defconstant uav_messages::TrajectoryTargetTestRequest::*ERR_NO_PATH_FOUND* 1)

(defun uav_messages::TrajectoryTargetTestRequest-to-symbol (const)
  (cond
        ((= const 0) 'uav_messages::TrajectoryTargetTestRequest::*OK*)
        ((= const 1) 'uav_messages::TrajectoryTargetTestRequest::*ERR_NO_PATH_FOUND*)
        (t nil)))

(defclass uav_messages::TrajectoryTargetTestRequest
  :super ros::object
  :slots (_start _end ))

(defmethod uav_messages::TrajectoryTargetTestRequest
  (:init
   (&key
    ((:start __start) (instance geometry_msgs::Point :init))
    ((:end __end) (instance geometry_msgs::Point :init))
    )
   (send-super :init)
   (setq _start __start)
   (setq _end __end)
   self)
  (:start
   (&rest __start)
   (if (keywordp (car __start))
       (send* _start __start)
     (progn
       (if __start (setq _start (car __start)))
       _start)))
  (:end
   (&rest __end)
   (if (keywordp (car __end))
       (send* _end __end)
     (progn
       (if __end (setq _end (car __end)))
       _end)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point _start
    (send _start :serialization-length)
    ;; geometry_msgs/Point _end
    (send _end :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point _start
       (send _start :serialize s)
     ;; geometry_msgs/Point _end
       (send _end :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point _start
     (send _start :deserialize buf ptr-) (incf ptr- (send _start :serialization-length))
   ;; geometry_msgs/Point _end
     (send _end :deserialize buf ptr-) (incf ptr- (send _end :serialization-length))
   ;;
   self)
  )

(defclass uav_messages::TrajectoryTargetTestResponse
  :super ros::object
  :slots (_success _error _runtime_us _traj ))

(defmethod uav_messages::TrajectoryTargetTestResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:error __error) 0)
    ((:runtime_us __runtime_us) 0)
    ((:traj __traj) (instance trajectory_msgs::MultiDOFJointTrajectory :init))
    )
   (send-super :init)
   (setq _success __success)
   (setq _error (round __error))
   (setq _runtime_us (round __runtime_us))
   (setq _traj __traj)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:error
   (&optional __error)
   (if __error (setq _error __error)) _error)
  (:runtime_us
   (&optional __runtime_us)
   (if __runtime_us (setq _runtime_us __runtime_us)) _runtime_us)
  (:traj
   (&rest __traj)
   (if (keywordp (car __traj))
       (send* _traj __traj)
     (progn
       (if __traj (setq _traj (car __traj)))
       _traj)))
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; uint8 _error
    1
    ;; uint32 _runtime_us
    4
    ;; trajectory_msgs/MultiDOFJointTrajectory _traj
    (send _traj :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; uint8 _error
       (write-byte _error s)
     ;; uint32 _runtime_us
       (write-long _runtime_us s)
     ;; trajectory_msgs/MultiDOFJointTrajectory _traj
       (send _traj :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8 _error
     (setq _error (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint32 _runtime_us
     (setq _runtime_us (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; trajectory_msgs/MultiDOFJointTrajectory _traj
     (send _traj :deserialize buf ptr-) (incf ptr- (send _traj :serialization-length))
   ;;
   self)
  )

(defclass uav_messages::TrajectoryTargetTest
  :super ros::object
  :slots ())

(setf (get uav_messages::TrajectoryTargetTest :md5sum-) "6aa66adc9f9e06718260a8729c57876c")
(setf (get uav_messages::TrajectoryTargetTest :datatype-) "uav_messages/TrajectoryTargetTest")
(setf (get uav_messages::TrajectoryTargetTest :request) uav_messages::TrajectoryTargetTestRequest)
(setf (get uav_messages::TrajectoryTargetTest :response) uav_messages::TrajectoryTargetTestResponse)

(defmethod uav_messages::TrajectoryTargetTestRequest
  (:response () (instance uav_messages::TrajectoryTargetTestResponse :init)))

(setf (get uav_messages::TrajectoryTargetTestRequest :md5sum-) "6aa66adc9f9e06718260a8729c57876c")
(setf (get uav_messages::TrajectoryTargetTestRequest :datatype-) "uav_messages/TrajectoryTargetTestRequest")
(setf (get uav_messages::TrajectoryTargetTestRequest :definition-)
      "uint8 OK=0
uint8 ERR_NO_PATH_FOUND=1

geometry_msgs/Point start
geometry_msgs/Point end

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
---
bool success
uint8 error
uint32 runtime_us
trajectory_msgs/MultiDOFJointTrajectory traj

================================================================================
MSG: trajectory_msgs/MultiDOFJointTrajectory
# The header is used to specify the coordinate frame and the reference time for the trajectory durations
Header header

# A representation of a multi-dof joint trajectory (each point is a transformation)
# Each point along the trajectory will include an array of positions/velocities/accelerations
# that has the same length as the array of joint names, and has the same order of joints as 
# the joint names array.

string[] joint_names
MultiDOFJointTrajectoryPoint[] points

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: trajectory_msgs/MultiDOFJointTrajectoryPoint
# Each multi-dof joint can specify a transform (up to 6 DOF)
geometry_msgs/Transform[] transforms

# There can be a velocity specified for the origin of the joint 
geometry_msgs/Twist[] velocities

# There can be an acceleration specified for the origin of the joint 
geometry_msgs/Twist[] accelerations

duration time_from_start

================================================================================
MSG: geometry_msgs/Transform
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation

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
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular
")

(setf (get uav_messages::TrajectoryTargetTestResponse :md5sum-) "6aa66adc9f9e06718260a8729c57876c")
(setf (get uav_messages::TrajectoryTargetTestResponse :datatype-) "uav_messages/TrajectoryTargetTestResponse")
(setf (get uav_messages::TrajectoryTargetTestResponse :definition-)
      "uint8 OK=0
uint8 ERR_NO_PATH_FOUND=1

geometry_msgs/Point start
geometry_msgs/Point end

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
---
bool success
uint8 error
uint32 runtime_us
trajectory_msgs/MultiDOFJointTrajectory traj

================================================================================
MSG: trajectory_msgs/MultiDOFJointTrajectory
# The header is used to specify the coordinate frame and the reference time for the trajectory durations
Header header

# A representation of a multi-dof joint trajectory (each point is a transformation)
# Each point along the trajectory will include an array of positions/velocities/accelerations
# that has the same length as the array of joint names, and has the same order of joints as 
# the joint names array.

string[] joint_names
MultiDOFJointTrajectoryPoint[] points

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: trajectory_msgs/MultiDOFJointTrajectoryPoint
# Each multi-dof joint can specify a transform (up to 6 DOF)
geometry_msgs/Transform[] transforms

# There can be a velocity specified for the origin of the joint 
geometry_msgs/Twist[] velocities

# There can be an acceleration specified for the origin of the joint 
geometry_msgs/Twist[] accelerations

duration time_from_start

================================================================================
MSG: geometry_msgs/Transform
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation

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
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular
")



(provide :uav_messages/TrajectoryTargetTest "6aa66adc9f9e06718260a8729c57876c")


