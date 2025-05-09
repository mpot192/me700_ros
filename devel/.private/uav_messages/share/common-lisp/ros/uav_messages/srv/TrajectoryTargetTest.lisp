; Auto-generated. Do not edit!


(cl:in-package uav_messages-srv)


;//! \htmlinclude TrajectoryTargetTest-request.msg.html

(cl:defclass <TrajectoryTargetTest-request> (roslisp-msg-protocol:ros-message)
  ((start
    :reader start
    :initarg :start
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (end
    :reader end
    :initarg :end
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass TrajectoryTargetTest-request (<TrajectoryTargetTest-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrajectoryTargetTest-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrajectoryTargetTest-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uav_messages-srv:<TrajectoryTargetTest-request> is deprecated: use uav_messages-srv:TrajectoryTargetTest-request instead.")))

(cl:ensure-generic-function 'start-val :lambda-list '(m))
(cl:defmethod start-val ((m <TrajectoryTargetTest-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_messages-srv:start-val is deprecated.  Use uav_messages-srv:start instead.")
  (start m))

(cl:ensure-generic-function 'end-val :lambda-list '(m))
(cl:defmethod end-val ((m <TrajectoryTargetTest-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_messages-srv:end-val is deprecated.  Use uav_messages-srv:end instead.")
  (end m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<TrajectoryTargetTest-request>)))
    "Constants for message type '<TrajectoryTargetTest-request>"
  '((:OK . 0)
    (:ERR_NO_PATH_FOUND . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'TrajectoryTargetTest-request)))
    "Constants for message type 'TrajectoryTargetTest-request"
  '((:OK . 0)
    (:ERR_NO_PATH_FOUND . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrajectoryTargetTest-request>) ostream)
  "Serializes a message object of type '<TrajectoryTargetTest-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'start) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'end) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrajectoryTargetTest-request>) istream)
  "Deserializes a message object of type '<TrajectoryTargetTest-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'start) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'end) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrajectoryTargetTest-request>)))
  "Returns string type for a service object of type '<TrajectoryTargetTest-request>"
  "uav_messages/TrajectoryTargetTestRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrajectoryTargetTest-request)))
  "Returns string type for a service object of type 'TrajectoryTargetTest-request"
  "uav_messages/TrajectoryTargetTestRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrajectoryTargetTest-request>)))
  "Returns md5sum for a message object of type '<TrajectoryTargetTest-request>"
  "6aa66adc9f9e06718260a8729c57876c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrajectoryTargetTest-request)))
  "Returns md5sum for a message object of type 'TrajectoryTargetTest-request"
  "6aa66adc9f9e06718260a8729c57876c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrajectoryTargetTest-request>)))
  "Returns full string definition for message of type '<TrajectoryTargetTest-request>"
  (cl:format cl:nil "uint8 OK=0~%uint8 ERR_NO_PATH_FOUND=1~%~%geometry_msgs/Point start~%geometry_msgs/Point end~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrajectoryTargetTest-request)))
  "Returns full string definition for message of type 'TrajectoryTargetTest-request"
  (cl:format cl:nil "uint8 OK=0~%uint8 ERR_NO_PATH_FOUND=1~%~%geometry_msgs/Point start~%geometry_msgs/Point end~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrajectoryTargetTest-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'start))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'end))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrajectoryTargetTest-request>))
  "Converts a ROS message object to a list"
  (cl:list 'TrajectoryTargetTest-request
    (cl:cons ':start (start msg))
    (cl:cons ':end (end msg))
))
;//! \htmlinclude TrajectoryTargetTest-response.msg.html

(cl:defclass <TrajectoryTargetTest-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (error
    :reader error
    :initarg :error
    :type cl:fixnum
    :initform 0)
   (runtime_us
    :reader runtime_us
    :initarg :runtime_us
    :type cl:integer
    :initform 0)
   (traj
    :reader traj
    :initarg :traj
    :type trajectory_msgs-msg:MultiDOFJointTrajectory
    :initform (cl:make-instance 'trajectory_msgs-msg:MultiDOFJointTrajectory)))
)

(cl:defclass TrajectoryTargetTest-response (<TrajectoryTargetTest-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrajectoryTargetTest-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrajectoryTargetTest-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uav_messages-srv:<TrajectoryTargetTest-response> is deprecated: use uav_messages-srv:TrajectoryTargetTest-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <TrajectoryTargetTest-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_messages-srv:success-val is deprecated.  Use uav_messages-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'error-val :lambda-list '(m))
(cl:defmethod error-val ((m <TrajectoryTargetTest-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_messages-srv:error-val is deprecated.  Use uav_messages-srv:error instead.")
  (error m))

(cl:ensure-generic-function 'runtime_us-val :lambda-list '(m))
(cl:defmethod runtime_us-val ((m <TrajectoryTargetTest-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_messages-srv:runtime_us-val is deprecated.  Use uav_messages-srv:runtime_us instead.")
  (runtime_us m))

(cl:ensure-generic-function 'traj-val :lambda-list '(m))
(cl:defmethod traj-val ((m <TrajectoryTargetTest-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_messages-srv:traj-val is deprecated.  Use uav_messages-srv:traj instead.")
  (traj m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrajectoryTargetTest-response>) ostream)
  "Serializes a message object of type '<TrajectoryTargetTest-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'error)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'runtime_us)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'runtime_us)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'runtime_us)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'runtime_us)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'traj) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrajectoryTargetTest-response>) istream)
  "Deserializes a message object of type '<TrajectoryTargetTest-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'error)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'runtime_us)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'runtime_us)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'runtime_us)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'runtime_us)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'traj) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrajectoryTargetTest-response>)))
  "Returns string type for a service object of type '<TrajectoryTargetTest-response>"
  "uav_messages/TrajectoryTargetTestResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrajectoryTargetTest-response)))
  "Returns string type for a service object of type 'TrajectoryTargetTest-response"
  "uav_messages/TrajectoryTargetTestResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrajectoryTargetTest-response>)))
  "Returns md5sum for a message object of type '<TrajectoryTargetTest-response>"
  "6aa66adc9f9e06718260a8729c57876c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrajectoryTargetTest-response)))
  "Returns md5sum for a message object of type 'TrajectoryTargetTest-response"
  "6aa66adc9f9e06718260a8729c57876c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrajectoryTargetTest-response>)))
  "Returns full string definition for message of type '<TrajectoryTargetTest-response>"
  (cl:format cl:nil "bool success~%uint8 error~%uint32 runtime_us~%trajectory_msgs/MultiDOFJointTrajectory traj~%~%================================================================================~%MSG: trajectory_msgs/MultiDOFJointTrajectory~%# The header is used to specify the coordinate frame and the reference time for the trajectory durations~%Header header~%~%# A representation of a multi-dof joint trajectory (each point is a transformation)~%# Each point along the trajectory will include an array of positions/velocities/accelerations~%# that has the same length as the array of joint names, and has the same order of joints as ~%# the joint names array.~%~%string[] joint_names~%MultiDOFJointTrajectoryPoint[] points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: trajectory_msgs/MultiDOFJointTrajectoryPoint~%# Each multi-dof joint can specify a transform (up to 6 DOF)~%geometry_msgs/Transform[] transforms~%~%# There can be a velocity specified for the origin of the joint ~%geometry_msgs/Twist[] velocities~%~%# There can be an acceleration specified for the origin of the joint ~%geometry_msgs/Twist[] accelerations~%~%duration time_from_start~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrajectoryTargetTest-response)))
  "Returns full string definition for message of type 'TrajectoryTargetTest-response"
  (cl:format cl:nil "bool success~%uint8 error~%uint32 runtime_us~%trajectory_msgs/MultiDOFJointTrajectory traj~%~%================================================================================~%MSG: trajectory_msgs/MultiDOFJointTrajectory~%# The header is used to specify the coordinate frame and the reference time for the trajectory durations~%Header header~%~%# A representation of a multi-dof joint trajectory (each point is a transformation)~%# Each point along the trajectory will include an array of positions/velocities/accelerations~%# that has the same length as the array of joint names, and has the same order of joints as ~%# the joint names array.~%~%string[] joint_names~%MultiDOFJointTrajectoryPoint[] points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: trajectory_msgs/MultiDOFJointTrajectoryPoint~%# Each multi-dof joint can specify a transform (up to 6 DOF)~%geometry_msgs/Transform[] transforms~%~%# There can be a velocity specified for the origin of the joint ~%geometry_msgs/Twist[] velocities~%~%# There can be an acceleration specified for the origin of the joint ~%geometry_msgs/Twist[] accelerations~%~%duration time_from_start~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrajectoryTargetTest-response>))
  (cl:+ 0
     1
     1
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'traj))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrajectoryTargetTest-response>))
  "Converts a ROS message object to a list"
  (cl:list 'TrajectoryTargetTest-response
    (cl:cons ':success (success msg))
    (cl:cons ':error (error msg))
    (cl:cons ':runtime_us (runtime_us msg))
    (cl:cons ':traj (traj msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'TrajectoryTargetTest)))
  'TrajectoryTargetTest-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'TrajectoryTargetTest)))
  'TrajectoryTargetTest-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrajectoryTargetTest)))
  "Returns string type for a service object of type '<TrajectoryTargetTest>"
  "uav_messages/TrajectoryTargetTest")