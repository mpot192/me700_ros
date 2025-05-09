;; Auto-generated. Do not edit!


(when (boundp 'uav_messages::Heartbeat)
  (if (not (find-package "UAV_MESSAGES"))
    (make-package "UAV_MESSAGES"))
  (shadow 'Heartbeat (find-package "UAV_MESSAGES")))
(unless (find-package "UAV_MESSAGES::HEARTBEAT")
  (make-package "UAV_MESSAGES::HEARTBEAT"))
(unless (find-package "UAV_MESSAGES::HEARTBEATREQUEST")
  (make-package "UAV_MESSAGES::HEARTBEATREQUEST"))
(unless (find-package "UAV_MESSAGES::HEARTBEATRESPONSE")
  (make-package "UAV_MESSAGES::HEARTBEATRESPONSE"))

(in-package "ROS")





(defclass uav_messages::HeartbeatRequest
  :super ros::object
  :slots (_heartbeat ))

(defmethod uav_messages::HeartbeatRequest
  (:init
   (&key
    ((:heartbeat __heartbeat) nil)
    )
   (send-super :init)
   (setq _heartbeat __heartbeat)
   self)
  (:heartbeat
   (&optional (__heartbeat :null))
   (if (not (eq __heartbeat :null)) (setq _heartbeat __heartbeat)) _heartbeat)
  (:serialization-length
   ()
   (+
    ;; bool _heartbeat
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _heartbeat
       (if _heartbeat (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _heartbeat
     (setq _heartbeat (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass uav_messages::HeartbeatResponse
  :super ros::object
  :slots (_heartbeat ))

(defmethod uav_messages::HeartbeatResponse
  (:init
   (&key
    ((:heartbeat __heartbeat) nil)
    )
   (send-super :init)
   (setq _heartbeat __heartbeat)
   self)
  (:heartbeat
   (&optional (__heartbeat :null))
   (if (not (eq __heartbeat :null)) (setq _heartbeat __heartbeat)) _heartbeat)
  (:serialization-length
   ()
   (+
    ;; bool _heartbeat
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _heartbeat
       (if _heartbeat (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _heartbeat
     (setq _heartbeat (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass uav_messages::Heartbeat
  :super ros::object
  :slots ())

(setf (get uav_messages::Heartbeat :md5sum-) "927048d99b53abc440d02d20ffdd561a")
(setf (get uav_messages::Heartbeat :datatype-) "uav_messages/Heartbeat")
(setf (get uav_messages::Heartbeat :request) uav_messages::HeartbeatRequest)
(setf (get uav_messages::Heartbeat :response) uav_messages::HeartbeatResponse)

(defmethod uav_messages::HeartbeatRequest
  (:response () (instance uav_messages::HeartbeatResponse :init)))

(setf (get uav_messages::HeartbeatRequest :md5sum-) "927048d99b53abc440d02d20ffdd561a")
(setf (get uav_messages::HeartbeatRequest :datatype-) "uav_messages/HeartbeatRequest")
(setf (get uav_messages::HeartbeatRequest :definition-)
      "bool heartbeat
---
bool heartbeat
")

(setf (get uav_messages::HeartbeatResponse :md5sum-) "927048d99b53abc440d02d20ffdd561a")
(setf (get uav_messages::HeartbeatResponse :datatype-) "uav_messages/HeartbeatResponse")
(setf (get uav_messages::HeartbeatResponse :definition-)
      "bool heartbeat
---
bool heartbeat
")



(provide :uav_messages/Heartbeat "927048d99b53abc440d02d20ffdd561a")


