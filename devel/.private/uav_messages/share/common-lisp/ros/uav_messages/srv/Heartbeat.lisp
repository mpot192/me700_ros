; Auto-generated. Do not edit!


(cl:in-package uav_messages-srv)


;//! \htmlinclude Heartbeat-request.msg.html

(cl:defclass <Heartbeat-request> (roslisp-msg-protocol:ros-message)
  ((heartbeat
    :reader heartbeat
    :initarg :heartbeat
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Heartbeat-request (<Heartbeat-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Heartbeat-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Heartbeat-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uav_messages-srv:<Heartbeat-request> is deprecated: use uav_messages-srv:Heartbeat-request instead.")))

(cl:ensure-generic-function 'heartbeat-val :lambda-list '(m))
(cl:defmethod heartbeat-val ((m <Heartbeat-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_messages-srv:heartbeat-val is deprecated.  Use uav_messages-srv:heartbeat instead.")
  (heartbeat m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Heartbeat-request>) ostream)
  "Serializes a message object of type '<Heartbeat-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'heartbeat) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Heartbeat-request>) istream)
  "Deserializes a message object of type '<Heartbeat-request>"
    (cl:setf (cl:slot-value msg 'heartbeat) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Heartbeat-request>)))
  "Returns string type for a service object of type '<Heartbeat-request>"
  "uav_messages/HeartbeatRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Heartbeat-request)))
  "Returns string type for a service object of type 'Heartbeat-request"
  "uav_messages/HeartbeatRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Heartbeat-request>)))
  "Returns md5sum for a message object of type '<Heartbeat-request>"
  "927048d99b53abc440d02d20ffdd561a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Heartbeat-request)))
  "Returns md5sum for a message object of type 'Heartbeat-request"
  "927048d99b53abc440d02d20ffdd561a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Heartbeat-request>)))
  "Returns full string definition for message of type '<Heartbeat-request>"
  (cl:format cl:nil "bool heartbeat~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Heartbeat-request)))
  "Returns full string definition for message of type 'Heartbeat-request"
  (cl:format cl:nil "bool heartbeat~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Heartbeat-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Heartbeat-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Heartbeat-request
    (cl:cons ':heartbeat (heartbeat msg))
))
;//! \htmlinclude Heartbeat-response.msg.html

(cl:defclass <Heartbeat-response> (roslisp-msg-protocol:ros-message)
  ((heartbeat
    :reader heartbeat
    :initarg :heartbeat
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Heartbeat-response (<Heartbeat-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Heartbeat-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Heartbeat-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uav_messages-srv:<Heartbeat-response> is deprecated: use uav_messages-srv:Heartbeat-response instead.")))

(cl:ensure-generic-function 'heartbeat-val :lambda-list '(m))
(cl:defmethod heartbeat-val ((m <Heartbeat-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_messages-srv:heartbeat-val is deprecated.  Use uav_messages-srv:heartbeat instead.")
  (heartbeat m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Heartbeat-response>) ostream)
  "Serializes a message object of type '<Heartbeat-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'heartbeat) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Heartbeat-response>) istream)
  "Deserializes a message object of type '<Heartbeat-response>"
    (cl:setf (cl:slot-value msg 'heartbeat) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Heartbeat-response>)))
  "Returns string type for a service object of type '<Heartbeat-response>"
  "uav_messages/HeartbeatResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Heartbeat-response)))
  "Returns string type for a service object of type 'Heartbeat-response"
  "uav_messages/HeartbeatResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Heartbeat-response>)))
  "Returns md5sum for a message object of type '<Heartbeat-response>"
  "927048d99b53abc440d02d20ffdd561a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Heartbeat-response)))
  "Returns md5sum for a message object of type 'Heartbeat-response"
  "927048d99b53abc440d02d20ffdd561a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Heartbeat-response>)))
  "Returns full string definition for message of type '<Heartbeat-response>"
  (cl:format cl:nil "bool heartbeat~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Heartbeat-response)))
  "Returns full string definition for message of type 'Heartbeat-response"
  (cl:format cl:nil "bool heartbeat~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Heartbeat-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Heartbeat-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Heartbeat-response
    (cl:cons ':heartbeat (heartbeat msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Heartbeat)))
  'Heartbeat-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Heartbeat)))
  'Heartbeat-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Heartbeat)))
  "Returns string type for a service object of type '<Heartbeat>"
  "uav_messages/Heartbeat")