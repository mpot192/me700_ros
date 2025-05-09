;; Auto-generated. Do not edit!


(when (boundp 'uav_messages::EsdfQuery)
  (if (not (find-package "UAV_MESSAGES"))
    (make-package "UAV_MESSAGES"))
  (shadow 'EsdfQuery (find-package "UAV_MESSAGES")))
(unless (find-package "UAV_MESSAGES::ESDFQUERY")
  (make-package "UAV_MESSAGES::ESDFQUERY"))
(unless (find-package "UAV_MESSAGES::ESDFQUERYREQUEST")
  (make-package "UAV_MESSAGES::ESDFQUERYREQUEST"))
(unless (find-package "UAV_MESSAGES::ESDFQUERYRESPONSE")
  (make-package "UAV_MESSAGES::ESDFQUERYRESPONSE"))

(in-package "ROS")





(defclass uav_messages::EsdfQueryRequest
  :super ros::object
  :slots (_x _y _z ))

(defmethod uav_messages::EsdfQueryRequest
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:serialization-length
   ()
   (+
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _z
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z
       (sys::poke _z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z
     (setq _z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass uav_messages::EsdfQueryResponse
  :super ros::object
  :slots (_distance ))

(defmethod uav_messages::EsdfQueryResponse
  (:init
   (&key
    ((:distance __distance) 0.0)
    )
   (send-super :init)
   (setq _distance (float __distance))
   self)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:serialization-length
   ()
   (+
    ;; float64 _distance
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _distance
       (sys::poke _distance (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _distance
     (setq _distance (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass uav_messages::EsdfQuery
  :super ros::object
  :slots ())

(setf (get uav_messages::EsdfQuery :md5sum-) "877c5238469713853bdc6a19780c7540")
(setf (get uav_messages::EsdfQuery :datatype-) "uav_messages/EsdfQuery")
(setf (get uav_messages::EsdfQuery :request) uav_messages::EsdfQueryRequest)
(setf (get uav_messages::EsdfQuery :response) uav_messages::EsdfQueryResponse)

(defmethod uav_messages::EsdfQueryRequest
  (:response () (instance uav_messages::EsdfQueryResponse :init)))

(setf (get uav_messages::EsdfQueryRequest :md5sum-) "877c5238469713853bdc6a19780c7540")
(setf (get uav_messages::EsdfQueryRequest :datatype-) "uav_messages/EsdfQueryRequest")
(setf (get uav_messages::EsdfQueryRequest :definition-)
      "float64 x
float64 y
float64 z
---
float64 distance
")

(setf (get uav_messages::EsdfQueryResponse :md5sum-) "877c5238469713853bdc6a19780c7540")
(setf (get uav_messages::EsdfQueryResponse :datatype-) "uav_messages/EsdfQueryResponse")
(setf (get uav_messages::EsdfQueryResponse :definition-)
      "float64 x
float64 y
float64 z
---
float64 distance
")



(provide :uav_messages/EsdfQuery "877c5238469713853bdc6a19780c7540")


