; Auto-generated. Do not edit!


(cl:in-package uav_messages-srv)


;//! \htmlinclude TrajectoryFollowerStatus-request.msg.html

(cl:defclass <TrajectoryFollowerStatus-request> (roslisp-msg-protocol:ros-message)
  ((request
    :reader request
    :initarg :request
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass TrajectoryFollowerStatus-request (<TrajectoryFollowerStatus-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrajectoryFollowerStatus-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrajectoryFollowerStatus-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uav_messages-srv:<TrajectoryFollowerStatus-request> is deprecated: use uav_messages-srv:TrajectoryFollowerStatus-request instead.")))

(cl:ensure-generic-function 'request-val :lambda-list '(m))
(cl:defmethod request-val ((m <TrajectoryFollowerStatus-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_messages-srv:request-val is deprecated.  Use uav_messages-srv:request instead.")
  (request m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<TrajectoryFollowerStatus-request>)))
    "Constants for message type '<TrajectoryFollowerStatus-request>"
  '((:IDLE . 0)
    (:FOLLOWING . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'TrajectoryFollowerStatus-request)))
    "Constants for message type 'TrajectoryFollowerStatus-request"
  '((:IDLE . 0)
    (:FOLLOWING . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrajectoryFollowerStatus-request>) ostream)
  "Serializes a message object of type '<TrajectoryFollowerStatus-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'request) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrajectoryFollowerStatus-request>) istream)
  "Deserializes a message object of type '<TrajectoryFollowerStatus-request>"
    (cl:setf (cl:slot-value msg 'request) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrajectoryFollowerStatus-request>)))
  "Returns string type for a service object of type '<TrajectoryFollowerStatus-request>"
  "uav_messages/TrajectoryFollowerStatusRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrajectoryFollowerStatus-request)))
  "Returns string type for a service object of type 'TrajectoryFollowerStatus-request"
  "uav_messages/TrajectoryFollowerStatusRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrajectoryFollowerStatus-request>)))
  "Returns md5sum for a message object of type '<TrajectoryFollowerStatus-request>"
  "29cf0115501ebd4739c6b692f76b7c0e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrajectoryFollowerStatus-request)))
  "Returns md5sum for a message object of type 'TrajectoryFollowerStatus-request"
  "29cf0115501ebd4739c6b692f76b7c0e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrajectoryFollowerStatus-request>)))
  "Returns full string definition for message of type '<TrajectoryFollowerStatus-request>"
  (cl:format cl:nil "uint8 IDLE=0~%uint8 FOLLOWING=1~%~%bool request~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrajectoryFollowerStatus-request)))
  "Returns full string definition for message of type 'TrajectoryFollowerStatus-request"
  (cl:format cl:nil "uint8 IDLE=0~%uint8 FOLLOWING=1~%~%bool request~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrajectoryFollowerStatus-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrajectoryFollowerStatus-request>))
  "Converts a ROS message object to a list"
  (cl:list 'TrajectoryFollowerStatus-request
    (cl:cons ':request (request msg))
))
;//! \htmlinclude TrajectoryFollowerStatus-response.msg.html

(cl:defclass <TrajectoryFollowerStatus-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0))
)

(cl:defclass TrajectoryFollowerStatus-response (<TrajectoryFollowerStatus-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrajectoryFollowerStatus-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrajectoryFollowerStatus-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uav_messages-srv:<TrajectoryFollowerStatus-response> is deprecated: use uav_messages-srv:TrajectoryFollowerStatus-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <TrajectoryFollowerStatus-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_messages-srv:status-val is deprecated.  Use uav_messages-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrajectoryFollowerStatus-response>) ostream)
  "Serializes a message object of type '<TrajectoryFollowerStatus-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrajectoryFollowerStatus-response>) istream)
  "Deserializes a message object of type '<TrajectoryFollowerStatus-response>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrajectoryFollowerStatus-response>)))
  "Returns string type for a service object of type '<TrajectoryFollowerStatus-response>"
  "uav_messages/TrajectoryFollowerStatusResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrajectoryFollowerStatus-response)))
  "Returns string type for a service object of type 'TrajectoryFollowerStatus-response"
  "uav_messages/TrajectoryFollowerStatusResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrajectoryFollowerStatus-response>)))
  "Returns md5sum for a message object of type '<TrajectoryFollowerStatus-response>"
  "29cf0115501ebd4739c6b692f76b7c0e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrajectoryFollowerStatus-response)))
  "Returns md5sum for a message object of type 'TrajectoryFollowerStatus-response"
  "29cf0115501ebd4739c6b692f76b7c0e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrajectoryFollowerStatus-response>)))
  "Returns full string definition for message of type '<TrajectoryFollowerStatus-response>"
  (cl:format cl:nil "uint8 status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrajectoryFollowerStatus-response)))
  "Returns full string definition for message of type 'TrajectoryFollowerStatus-response"
  (cl:format cl:nil "uint8 status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrajectoryFollowerStatus-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrajectoryFollowerStatus-response>))
  "Converts a ROS message object to a list"
  (cl:list 'TrajectoryFollowerStatus-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'TrajectoryFollowerStatus)))
  'TrajectoryFollowerStatus-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'TrajectoryFollowerStatus)))
  'TrajectoryFollowerStatus-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrajectoryFollowerStatus)))
  "Returns string type for a service object of type '<TrajectoryFollowerStatus>"
  "uav_messages/TrajectoryFollowerStatus")