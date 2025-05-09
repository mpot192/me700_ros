; Auto-generated. Do not edit!


(cl:in-package uav_messages-msg)


;//! \htmlinclude LeftRightRow.msg.html

(cl:defclass <LeftRightRow> (roslisp-msg-protocol:ros-message)
  ((left_r
    :reader left_r
    :initarg :left_r
    :type cl:float
    :initform 0.0)
   (left_theta
    :reader left_theta
    :initarg :left_theta
    :type cl:float
    :initform 0.0)
   (right_r
    :reader right_r
    :initarg :right_r
    :type cl:float
    :initform 0.0)
   (right_theta
    :reader right_theta
    :initarg :right_theta
    :type cl:float
    :initform 0.0))
)

(cl:defclass LeftRightRow (<LeftRightRow>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LeftRightRow>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LeftRightRow)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uav_messages-msg:<LeftRightRow> is deprecated: use uav_messages-msg:LeftRightRow instead.")))

(cl:ensure-generic-function 'left_r-val :lambda-list '(m))
(cl:defmethod left_r-val ((m <LeftRightRow>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_messages-msg:left_r-val is deprecated.  Use uav_messages-msg:left_r instead.")
  (left_r m))

(cl:ensure-generic-function 'left_theta-val :lambda-list '(m))
(cl:defmethod left_theta-val ((m <LeftRightRow>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_messages-msg:left_theta-val is deprecated.  Use uav_messages-msg:left_theta instead.")
  (left_theta m))

(cl:ensure-generic-function 'right_r-val :lambda-list '(m))
(cl:defmethod right_r-val ((m <LeftRightRow>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_messages-msg:right_r-val is deprecated.  Use uav_messages-msg:right_r instead.")
  (right_r m))

(cl:ensure-generic-function 'right_theta-val :lambda-list '(m))
(cl:defmethod right_theta-val ((m <LeftRightRow>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_messages-msg:right_theta-val is deprecated.  Use uav_messages-msg:right_theta instead.")
  (right_theta m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LeftRightRow>) ostream)
  "Serializes a message object of type '<LeftRightRow>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'left_r))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'left_theta))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'right_r))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'right_theta))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LeftRightRow>) istream)
  "Deserializes a message object of type '<LeftRightRow>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_r) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_theta) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_r) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_theta) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LeftRightRow>)))
  "Returns string type for a message object of type '<LeftRightRow>"
  "uav_messages/LeftRightRow")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LeftRightRow)))
  "Returns string type for a message object of type 'LeftRightRow"
  "uav_messages/LeftRightRow")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LeftRightRow>)))
  "Returns md5sum for a message object of type '<LeftRightRow>"
  "9547fcc8a1d9006ddee6af9894e1d3ac")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LeftRightRow)))
  "Returns md5sum for a message object of type 'LeftRightRow"
  "9547fcc8a1d9006ddee6af9894e1d3ac")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LeftRightRow>)))
  "Returns full string definition for message of type '<LeftRightRow>"
  (cl:format cl:nil "# Most likely left/right lines in radius/angle formation.~%float64 left_r~%float64 left_theta~%float64 right_r~%float64 right_theta~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LeftRightRow)))
  "Returns full string definition for message of type 'LeftRightRow"
  (cl:format cl:nil "# Most likely left/right lines in radius/angle formation.~%float64 left_r~%float64 left_theta~%float64 right_r~%float64 right_theta~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LeftRightRow>))
  (cl:+ 0
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LeftRightRow>))
  "Converts a ROS message object to a list"
  (cl:list 'LeftRightRow
    (cl:cons ':left_r (left_r msg))
    (cl:cons ':left_theta (left_theta msg))
    (cl:cons ':right_r (right_r msg))
    (cl:cons ':right_theta (right_theta msg))
))
