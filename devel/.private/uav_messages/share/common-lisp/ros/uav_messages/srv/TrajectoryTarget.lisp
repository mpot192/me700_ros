; Auto-generated. Do not edit!


(cl:in-package uav_messages-srv)


;//! \htmlinclude TrajectoryTarget-request.msg.html

(cl:defclass <TrajectoryTarget-request> (roslisp-msg-protocol:ros-message)
  ((waypoints
    :reader waypoints
    :initarg :waypoints
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point)))
   (start_velocity
    :reader start_velocity
    :initarg :start_velocity
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (end_velocity
    :reader end_velocity
    :initarg :end_velocity
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (target_velocity
    :reader target_velocity
    :initarg :target_velocity
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3)))
)

(cl:defclass TrajectoryTarget-request (<TrajectoryTarget-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrajectoryTarget-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrajectoryTarget-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uav_messages-srv:<TrajectoryTarget-request> is deprecated: use uav_messages-srv:TrajectoryTarget-request instead.")))

(cl:ensure-generic-function 'waypoints-val :lambda-list '(m))
(cl:defmethod waypoints-val ((m <TrajectoryTarget-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_messages-srv:waypoints-val is deprecated.  Use uav_messages-srv:waypoints instead.")
  (waypoints m))

(cl:ensure-generic-function 'start_velocity-val :lambda-list '(m))
(cl:defmethod start_velocity-val ((m <TrajectoryTarget-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_messages-srv:start_velocity-val is deprecated.  Use uav_messages-srv:start_velocity instead.")
  (start_velocity m))

(cl:ensure-generic-function 'end_velocity-val :lambda-list '(m))
(cl:defmethod end_velocity-val ((m <TrajectoryTarget-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_messages-srv:end_velocity-val is deprecated.  Use uav_messages-srv:end_velocity instead.")
  (end_velocity m))

(cl:ensure-generic-function 'target_velocity-val :lambda-list '(m))
(cl:defmethod target_velocity-val ((m <TrajectoryTarget-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_messages-srv:target_velocity-val is deprecated.  Use uav_messages-srv:target_velocity instead.")
  (target_velocity m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<TrajectoryTarget-request>)))
    "Constants for message type '<TrajectoryTarget-request>"
  '((:OK . 0)
    (:ERR_NO_PATH_FOUND . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'TrajectoryTarget-request)))
    "Constants for message type 'TrajectoryTarget-request"
  '((:OK . 0)
    (:ERR_NO_PATH_FOUND . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrajectoryTarget-request>) ostream)
  "Serializes a message object of type '<TrajectoryTarget-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'waypoints))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'waypoints))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'start_velocity) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'end_velocity) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'target_velocity) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrajectoryTarget-request>) istream)
  "Deserializes a message object of type '<TrajectoryTarget-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'waypoints) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'waypoints)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'start_velocity) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'end_velocity) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'target_velocity) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrajectoryTarget-request>)))
  "Returns string type for a service object of type '<TrajectoryTarget-request>"
  "uav_messages/TrajectoryTargetRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrajectoryTarget-request)))
  "Returns string type for a service object of type 'TrajectoryTarget-request"
  "uav_messages/TrajectoryTargetRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrajectoryTarget-request>)))
  "Returns md5sum for a message object of type '<TrajectoryTarget-request>"
  "ccd1ece0eed8a932ade7a0f997a2a505")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrajectoryTarget-request)))
  "Returns md5sum for a message object of type 'TrajectoryTarget-request"
  "ccd1ece0eed8a932ade7a0f997a2a505")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrajectoryTarget-request>)))
  "Returns full string definition for message of type '<TrajectoryTarget-request>"
  (cl:format cl:nil "uint8 OK=0~%uint8 ERR_NO_PATH_FOUND=1~%~%geometry_msgs/Point[] waypoints~%geometry_msgs/Vector3 start_velocity~%geometry_msgs/Vector3 end_velocity~%geometry_msgs/Vector3 target_velocity~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrajectoryTarget-request)))
  "Returns full string definition for message of type 'TrajectoryTarget-request"
  (cl:format cl:nil "uint8 OK=0~%uint8 ERR_NO_PATH_FOUND=1~%~%geometry_msgs/Point[] waypoints~%geometry_msgs/Vector3 start_velocity~%geometry_msgs/Vector3 end_velocity~%geometry_msgs/Vector3 target_velocity~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrajectoryTarget-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'waypoints) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'start_velocity))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'end_velocity))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'target_velocity))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrajectoryTarget-request>))
  "Converts a ROS message object to a list"
  (cl:list 'TrajectoryTarget-request
    (cl:cons ':waypoints (waypoints msg))
    (cl:cons ':start_velocity (start_velocity msg))
    (cl:cons ':end_velocity (end_velocity msg))
    (cl:cons ':target_velocity (target_velocity msg))
))
;//! \htmlinclude TrajectoryTarget-response.msg.html

(cl:defclass <TrajectoryTarget-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (error
    :reader error
    :initarg :error
    :type cl:fixnum
    :initform 0))
)

(cl:defclass TrajectoryTarget-response (<TrajectoryTarget-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrajectoryTarget-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrajectoryTarget-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uav_messages-srv:<TrajectoryTarget-response> is deprecated: use uav_messages-srv:TrajectoryTarget-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <TrajectoryTarget-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_messages-srv:success-val is deprecated.  Use uav_messages-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'error-val :lambda-list '(m))
(cl:defmethod error-val ((m <TrajectoryTarget-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_messages-srv:error-val is deprecated.  Use uav_messages-srv:error instead.")
  (error m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrajectoryTarget-response>) ostream)
  "Serializes a message object of type '<TrajectoryTarget-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'error)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrajectoryTarget-response>) istream)
  "Deserializes a message object of type '<TrajectoryTarget-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'error)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrajectoryTarget-response>)))
  "Returns string type for a service object of type '<TrajectoryTarget-response>"
  "uav_messages/TrajectoryTargetResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrajectoryTarget-response)))
  "Returns string type for a service object of type 'TrajectoryTarget-response"
  "uav_messages/TrajectoryTargetResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrajectoryTarget-response>)))
  "Returns md5sum for a message object of type '<TrajectoryTarget-response>"
  "ccd1ece0eed8a932ade7a0f997a2a505")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrajectoryTarget-response)))
  "Returns md5sum for a message object of type 'TrajectoryTarget-response"
  "ccd1ece0eed8a932ade7a0f997a2a505")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrajectoryTarget-response>)))
  "Returns full string definition for message of type '<TrajectoryTarget-response>"
  (cl:format cl:nil "bool success~%uint8 error~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrajectoryTarget-response)))
  "Returns full string definition for message of type 'TrajectoryTarget-response"
  (cl:format cl:nil "bool success~%uint8 error~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrajectoryTarget-response>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrajectoryTarget-response>))
  "Converts a ROS message object to a list"
  (cl:list 'TrajectoryTarget-response
    (cl:cons ':success (success msg))
    (cl:cons ':error (error msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'TrajectoryTarget)))
  'TrajectoryTarget-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'TrajectoryTarget)))
  'TrajectoryTarget-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrajectoryTarget)))
  "Returns string type for a service object of type '<TrajectoryTarget>"
  "uav_messages/TrajectoryTarget")