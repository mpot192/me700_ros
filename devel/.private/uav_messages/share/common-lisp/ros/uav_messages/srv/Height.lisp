; Auto-generated. Do not edit!


(cl:in-package uav_messages-srv)


;//! \htmlinclude Height-request.msg.html

(cl:defclass <Height-request> (roslisp-msg-protocol:ros-message)
  ((points
    :reader points
    :initarg :points
    :type (cl:vector geometry_msgs-msg:Vector3)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Vector3 :initial-element (cl:make-instance 'geometry_msgs-msg:Vector3))))
)

(cl:defclass Height-request (<Height-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Height-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Height-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uav_messages-srv:<Height-request> is deprecated: use uav_messages-srv:Height-request instead.")))

(cl:ensure-generic-function 'points-val :lambda-list '(m))
(cl:defmethod points-val ((m <Height-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_messages-srv:points-val is deprecated.  Use uav_messages-srv:points instead.")
  (points m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Height-request>)))
    "Constants for message type '<Height-request>"
  '((:OK . 0)
    (:SOME_INVALID . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Height-request)))
    "Constants for message type 'Height-request"
  '((:OK . 0)
    (:SOME_INVALID . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Height-request>) ostream)
  "Serializes a message object of type '<Height-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'points))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Height-request>) istream)
  "Deserializes a message object of type '<Height-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Vector3))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Height-request>)))
  "Returns string type for a service object of type '<Height-request>"
  "uav_messages/HeightRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Height-request)))
  "Returns string type for a service object of type 'Height-request"
  "uav_messages/HeightRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Height-request>)))
  "Returns md5sum for a message object of type '<Height-request>"
  "af6c894dddf3f73449d8d258ec820c83")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Height-request)))
  "Returns md5sum for a message object of type 'Height-request"
  "af6c894dddf3f73449d8d258ec820c83")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Height-request>)))
  "Returns full string definition for message of type '<Height-request>"
  (cl:format cl:nil "uint8 OK=0~%uint8 SOME_INVALID=1~%~%geometry_msgs/Vector3[] points~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Height-request)))
  "Returns full string definition for message of type 'Height-request"
  (cl:format cl:nil "uint8 OK=0~%uint8 SOME_INVALID=1~%~%geometry_msgs/Vector3[] points~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Height-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Height-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Height-request
    (cl:cons ':points (points msg))
))
;//! \htmlinclude Height-response.msg.html

(cl:defclass <Height-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0)
   (valid
    :reader valid
    :initarg :valid
    :type (cl:vector cl:boolean)
   :initform (cl:make-array 0 :element-type 'cl:boolean :initial-element cl:nil))
   (heights
    :reader heights
    :initarg :heights
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (mean
    :reader mean
    :initarg :mean
    :type cl:float
    :initform 0.0))
)

(cl:defclass Height-response (<Height-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Height-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Height-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uav_messages-srv:<Height-response> is deprecated: use uav_messages-srv:Height-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <Height-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_messages-srv:status-val is deprecated.  Use uav_messages-srv:status instead.")
  (status m))

(cl:ensure-generic-function 'valid-val :lambda-list '(m))
(cl:defmethod valid-val ((m <Height-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_messages-srv:valid-val is deprecated.  Use uav_messages-srv:valid instead.")
  (valid m))

(cl:ensure-generic-function 'heights-val :lambda-list '(m))
(cl:defmethod heights-val ((m <Height-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_messages-srv:heights-val is deprecated.  Use uav_messages-srv:heights instead.")
  (heights m))

(cl:ensure-generic-function 'mean-val :lambda-list '(m))
(cl:defmethod mean-val ((m <Height-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_messages-srv:mean-val is deprecated.  Use uav_messages-srv:mean instead.")
  (mean m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Height-response>) ostream)
  "Serializes a message object of type '<Height-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'valid))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if ele 1 0)) ostream))
   (cl:slot-value msg 'valid))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'heights))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'heights))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'mean))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Height-response>) istream)
  "Deserializes a message object of type '<Height-response>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'valid) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'valid)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:not (cl:zerop (cl:read-byte istream)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'heights) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'heights)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'mean) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Height-response>)))
  "Returns string type for a service object of type '<Height-response>"
  "uav_messages/HeightResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Height-response)))
  "Returns string type for a service object of type 'Height-response"
  "uav_messages/HeightResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Height-response>)))
  "Returns md5sum for a message object of type '<Height-response>"
  "af6c894dddf3f73449d8d258ec820c83")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Height-response)))
  "Returns md5sum for a message object of type 'Height-response"
  "af6c894dddf3f73449d8d258ec820c83")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Height-response>)))
  "Returns full string definition for message of type '<Height-response>"
  (cl:format cl:nil "uint8 status~%bool[] valid~%float64[] heights~%float64 mean~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Height-response)))
  "Returns full string definition for message of type 'Height-response"
  (cl:format cl:nil "uint8 status~%bool[] valid~%float64[] heights~%float64 mean~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Height-response>))
  (cl:+ 0
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'valid) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'heights) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Height-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Height-response
    (cl:cons ':status (status msg))
    (cl:cons ':valid (valid msg))
    (cl:cons ':heights (heights msg))
    (cl:cons ':mean (mean msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Height)))
  'Height-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Height)))
  'Height-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Height)))
  "Returns string type for a service object of type '<Height>"
  "uav_messages/Height")