;; Auto-generated. Do not edit!


(when (boundp 'uav_messages::TrajectoryFollowerStatus)
  (if (not (find-package "UAV_MESSAGES"))
    (make-package "UAV_MESSAGES"))
  (shadow 'TrajectoryFollowerStatus (find-package "UAV_MESSAGES")))
(unless (find-package "UAV_MESSAGES::TRAJECTORYFOLLOWERSTATUS")
  (make-package "UAV_MESSAGES::TRAJECTORYFOLLOWERSTATUS"))
(unless (find-package "UAV_MESSAGES::TRAJECTORYFOLLOWERSTATUSREQUEST")
  (make-package "UAV_MESSAGES::TRAJECTORYFOLLOWERSTATUSREQUEST"))
(unless (find-package "UAV_MESSAGES::TRAJECTORYFOLLOWERSTATUSRESPONSE")
  (make-package "UAV_MESSAGES::TRAJECTORYFOLLOWERSTATUSRESPONSE"))

(in-package "ROS")





(intern "*IDLE*" (find-package "UAV_MESSAGES::TRAJECTORYFOLLOWERSTATUSREQUEST"))
(shadow '*IDLE* (find-package "UAV_MESSAGES::TRAJECTORYFOLLOWERSTATUSREQUEST"))
(defconstant uav_messages::TrajectoryFollowerStatusRequest::*IDLE* 0)
(intern "*FOLLOWING*" (find-package "UAV_MESSAGES::TRAJECTORYFOLLOWERSTATUSREQUEST"))
(shadow '*FOLLOWING* (find-package "UAV_MESSAGES::TRAJECTORYFOLLOWERSTATUSREQUEST"))
(defconstant uav_messages::TrajectoryFollowerStatusRequest::*FOLLOWING* 1)

(defun uav_messages::TrajectoryFollowerStatusRequest-to-symbol (const)
  (cond
        ((= const 0) 'uav_messages::TrajectoryFollowerStatusRequest::*IDLE*)
        ((= const 1) 'uav_messages::TrajectoryFollowerStatusRequest::*FOLLOWING*)
        (t nil)))

(defclass uav_messages::TrajectoryFollowerStatusRequest
  :super ros::object
  :slots (_request ))

(defmethod uav_messages::TrajectoryFollowerStatusRequest
  (:init
   (&key
    ((:request __request) nil)
    )
   (send-super :init)
   (setq _request __request)
   self)
  (:request
   (&optional (__request :null))
   (if (not (eq __request :null)) (setq _request __request)) _request)
  (:serialization-length
   ()
   (+
    ;; bool _request
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _request
       (if _request (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _request
     (setq _request (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass uav_messages::TrajectoryFollowerStatusResponse
  :super ros::object
  :slots (_status ))

(defmethod uav_messages::TrajectoryFollowerStatusResponse
  (:init
   (&key
    ((:status __status) 0)
    )
   (send-super :init)
   (setq _status (round __status))
   self)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; uint8 _status
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _status
       (write-byte _status s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _status
     (setq _status (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass uav_messages::TrajectoryFollowerStatus
  :super ros::object
  :slots ())

(setf (get uav_messages::TrajectoryFollowerStatus :md5sum-) "29cf0115501ebd4739c6b692f76b7c0e")
(setf (get uav_messages::TrajectoryFollowerStatus :datatype-) "uav_messages/TrajectoryFollowerStatus")
(setf (get uav_messages::TrajectoryFollowerStatus :request) uav_messages::TrajectoryFollowerStatusRequest)
(setf (get uav_messages::TrajectoryFollowerStatus :response) uav_messages::TrajectoryFollowerStatusResponse)

(defmethod uav_messages::TrajectoryFollowerStatusRequest
  (:response () (instance uav_messages::TrajectoryFollowerStatusResponse :init)))

(setf (get uav_messages::TrajectoryFollowerStatusRequest :md5sum-) "29cf0115501ebd4739c6b692f76b7c0e")
(setf (get uav_messages::TrajectoryFollowerStatusRequest :datatype-) "uav_messages/TrajectoryFollowerStatusRequest")
(setf (get uav_messages::TrajectoryFollowerStatusRequest :definition-)
      "uint8 IDLE=0
uint8 FOLLOWING=1

bool request
---
uint8 status
")

(setf (get uav_messages::TrajectoryFollowerStatusResponse :md5sum-) "29cf0115501ebd4739c6b692f76b7c0e")
(setf (get uav_messages::TrajectoryFollowerStatusResponse :datatype-) "uav_messages/TrajectoryFollowerStatusResponse")
(setf (get uav_messages::TrajectoryFollowerStatusResponse :definition-)
      "uint8 IDLE=0
uint8 FOLLOWING=1

bool request
---
uint8 status
")



(provide :uav_messages/TrajectoryFollowerStatus "29cf0115501ebd4739c6b692f76b7c0e")


