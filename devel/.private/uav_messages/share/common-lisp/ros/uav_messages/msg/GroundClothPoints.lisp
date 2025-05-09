; Auto-generated. Do not edit!


(cl:in-package uav_messages-msg)


;//! \htmlinclude GroundClothPoints.msg.html

(cl:defclass <GroundClothPoints> (roslisp-msg-protocol:ros-message)
  ((origin
    :reader origin
    :initarg :origin
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (width
    :reader width
    :initarg :width
    :type cl:integer
    :initform 0)
   (length
    :reader length
    :initarg :length
    :type cl:integer
    :initform 0)
   (resolution
    :reader resolution
    :initarg :resolution
    :type cl:float
    :initform 0.0)
   (heights
    :reader heights
    :initarg :heights
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass GroundClothPoints (<GroundClothPoints>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GroundClothPoints>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GroundClothPoints)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uav_messages-msg:<GroundClothPoints> is deprecated: use uav_messages-msg:GroundClothPoints instead.")))

(cl:ensure-generic-function 'origin-val :lambda-list '(m))
(cl:defmethod origin-val ((m <GroundClothPoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_messages-msg:origin-val is deprecated.  Use uav_messages-msg:origin instead.")
  (origin m))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <GroundClothPoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_messages-msg:width-val is deprecated.  Use uav_messages-msg:width instead.")
  (width m))

(cl:ensure-generic-function 'length-val :lambda-list '(m))
(cl:defmethod length-val ((m <GroundClothPoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_messages-msg:length-val is deprecated.  Use uav_messages-msg:length instead.")
  (length m))

(cl:ensure-generic-function 'resolution-val :lambda-list '(m))
(cl:defmethod resolution-val ((m <GroundClothPoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_messages-msg:resolution-val is deprecated.  Use uav_messages-msg:resolution instead.")
  (resolution m))

(cl:ensure-generic-function 'heights-val :lambda-list '(m))
(cl:defmethod heights-val ((m <GroundClothPoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_messages-msg:heights-val is deprecated.  Use uav_messages-msg:heights instead.")
  (heights m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GroundClothPoints>) ostream)
  "Serializes a message object of type '<GroundClothPoints>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'origin) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'length)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'length)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'length)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'length)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'resolution))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
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
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GroundClothPoints>) istream)
  "Deserializes a message object of type '<GroundClothPoints>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'origin) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'length)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'length)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'length)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'length)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'resolution) (roslisp-utils:decode-double-float-bits bits)))
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GroundClothPoints>)))
  "Returns string type for a message object of type '<GroundClothPoints>"
  "uav_messages/GroundClothPoints")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GroundClothPoints)))
  "Returns string type for a message object of type 'GroundClothPoints"
  "uav_messages/GroundClothPoints")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GroundClothPoints>)))
  "Returns md5sum for a message object of type '<GroundClothPoints>"
  "38fbc55ea331f440db17b20331203f67")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GroundClothPoints)))
  "Returns md5sum for a message object of type 'GroundClothPoints"
  "38fbc55ea331f440db17b20331203f67")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GroundClothPoints>)))
  "Returns full string definition for message of type '<GroundClothPoints>"
  (cl:format cl:nil "# Data alignment origin~%geometry_msgs/Vector3 origin~%uint32 width~%uint32 length~%float64 resolution~%float64[] heights~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GroundClothPoints)))
  "Returns full string definition for message of type 'GroundClothPoints"
  (cl:format cl:nil "# Data alignment origin~%geometry_msgs/Vector3 origin~%uint32 width~%uint32 length~%float64 resolution~%float64[] heights~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GroundClothPoints>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'origin))
     4
     4
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'heights) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GroundClothPoints>))
  "Converts a ROS message object to a list"
  (cl:list 'GroundClothPoints
    (cl:cons ':origin (origin msg))
    (cl:cons ':width (width msg))
    (cl:cons ':length (length msg))
    (cl:cons ':resolution (resolution msg))
    (cl:cons ':heights (heights msg))
))
