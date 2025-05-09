; Auto-generated. Do not edit!


(cl:in-package uav_messages-srv)


;//! \htmlinclude EsdfQuery-request.msg.html

(cl:defclass <EsdfQuery-request> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (z
    :reader z
    :initarg :z
    :type cl:float
    :initform 0.0))
)

(cl:defclass EsdfQuery-request (<EsdfQuery-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EsdfQuery-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EsdfQuery-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uav_messages-srv:<EsdfQuery-request> is deprecated: use uav_messages-srv:EsdfQuery-request instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <EsdfQuery-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_messages-srv:x-val is deprecated.  Use uav_messages-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <EsdfQuery-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_messages-srv:y-val is deprecated.  Use uav_messages-srv:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <EsdfQuery-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_messages-srv:z-val is deprecated.  Use uav_messages-srv:z instead.")
  (z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EsdfQuery-request>) ostream)
  "Serializes a message object of type '<EsdfQuery-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EsdfQuery-request>) istream)
  "Deserializes a message object of type '<EsdfQuery-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EsdfQuery-request>)))
  "Returns string type for a service object of type '<EsdfQuery-request>"
  "uav_messages/EsdfQueryRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EsdfQuery-request)))
  "Returns string type for a service object of type 'EsdfQuery-request"
  "uav_messages/EsdfQueryRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EsdfQuery-request>)))
  "Returns md5sum for a message object of type '<EsdfQuery-request>"
  "877c5238469713853bdc6a19780c7540")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EsdfQuery-request)))
  "Returns md5sum for a message object of type 'EsdfQuery-request"
  "877c5238469713853bdc6a19780c7540")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EsdfQuery-request>)))
  "Returns full string definition for message of type '<EsdfQuery-request>"
  (cl:format cl:nil "float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EsdfQuery-request)))
  "Returns full string definition for message of type 'EsdfQuery-request"
  (cl:format cl:nil "float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EsdfQuery-request>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EsdfQuery-request>))
  "Converts a ROS message object to a list"
  (cl:list 'EsdfQuery-request
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
))
;//! \htmlinclude EsdfQuery-response.msg.html

(cl:defclass <EsdfQuery-response> (roslisp-msg-protocol:ros-message)
  ((distance
    :reader distance
    :initarg :distance
    :type cl:float
    :initform 0.0))
)

(cl:defclass EsdfQuery-response (<EsdfQuery-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EsdfQuery-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EsdfQuery-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uav_messages-srv:<EsdfQuery-response> is deprecated: use uav_messages-srv:EsdfQuery-response instead.")))

(cl:ensure-generic-function 'distance-val :lambda-list '(m))
(cl:defmethod distance-val ((m <EsdfQuery-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_messages-srv:distance-val is deprecated.  Use uav_messages-srv:distance instead.")
  (distance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EsdfQuery-response>) ostream)
  "Serializes a message object of type '<EsdfQuery-response>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EsdfQuery-response>) istream)
  "Deserializes a message object of type '<EsdfQuery-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distance) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EsdfQuery-response>)))
  "Returns string type for a service object of type '<EsdfQuery-response>"
  "uav_messages/EsdfQueryResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EsdfQuery-response)))
  "Returns string type for a service object of type 'EsdfQuery-response"
  "uav_messages/EsdfQueryResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EsdfQuery-response>)))
  "Returns md5sum for a message object of type '<EsdfQuery-response>"
  "877c5238469713853bdc6a19780c7540")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EsdfQuery-response)))
  "Returns md5sum for a message object of type 'EsdfQuery-response"
  "877c5238469713853bdc6a19780c7540")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EsdfQuery-response>)))
  "Returns full string definition for message of type '<EsdfQuery-response>"
  (cl:format cl:nil "float64 distance~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EsdfQuery-response)))
  "Returns full string definition for message of type 'EsdfQuery-response"
  (cl:format cl:nil "float64 distance~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EsdfQuery-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EsdfQuery-response>))
  "Converts a ROS message object to a list"
  (cl:list 'EsdfQuery-response
    (cl:cons ':distance (distance msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'EsdfQuery)))
  'EsdfQuery-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'EsdfQuery)))
  'EsdfQuery-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EsdfQuery)))
  "Returns string type for a service object of type '<EsdfQuery>"
  "uav_messages/EsdfQuery")