;; Auto-generated. Do not edit!


(when (boundp 'uav_messages::Height)
  (if (not (find-package "UAV_MESSAGES"))
    (make-package "UAV_MESSAGES"))
  (shadow 'Height (find-package "UAV_MESSAGES")))
(unless (find-package "UAV_MESSAGES::HEIGHT")
  (make-package "UAV_MESSAGES::HEIGHT"))
(unless (find-package "UAV_MESSAGES::HEIGHTREQUEST")
  (make-package "UAV_MESSAGES::HEIGHTREQUEST"))
(unless (find-package "UAV_MESSAGES::HEIGHTRESPONSE")
  (make-package "UAV_MESSAGES::HEIGHTRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(intern "*OK*" (find-package "UAV_MESSAGES::HEIGHTREQUEST"))
(shadow '*OK* (find-package "UAV_MESSAGES::HEIGHTREQUEST"))
(defconstant uav_messages::HeightRequest::*OK* 0)
(intern "*SOME_INVALID*" (find-package "UAV_MESSAGES::HEIGHTREQUEST"))
(shadow '*SOME_INVALID* (find-package "UAV_MESSAGES::HEIGHTREQUEST"))
(defconstant uav_messages::HeightRequest::*SOME_INVALID* 1)

(defun uav_messages::HeightRequest-to-symbol (const)
  (cond
        ((= const 0) 'uav_messages::HeightRequest::*OK*)
        ((= const 1) 'uav_messages::HeightRequest::*SOME_INVALID*)
        (t nil)))

(defclass uav_messages::HeightRequest
  :super ros::object
  :slots (_points ))

(defmethod uav_messages::HeightRequest
  (:init
   (&key
    ((:points __points) ())
    )
   (send-super :init)
   (setq _points __points)
   self)
  (:points
   (&rest __points)
   (if (keywordp (car __points))
       (send* _points __points)
     (progn
       (if __points (setq _points (car __points)))
       _points)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Vector3[] _points
    (apply #'+ (send-all _points :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Vector3[] _points
     (write-long (length _points) s)
     (dolist (elem _points)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Vector3[] _points
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _points (let (r) (dotimes (i n) (push (instance geometry_msgs::Vector3 :init) r)) r))
     (dolist (elem- _points)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(defclass uav_messages::HeightResponse
  :super ros::object
  :slots (_status _valid _heights _mean ))

(defmethod uav_messages::HeightResponse
  (:init
   (&key
    ((:status __status) 0)
    ((:valid __valid) (let (r) (dotimes (i 0) (push nil r)) r))
    ((:heights __heights) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:mean __mean) 0.0)
    )
   (send-super :init)
   (setq _status (round __status))
   (setq _valid __valid)
   (setq _heights __heights)
   (setq _mean (float __mean))
   self)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:valid
   (&optional __valid)
   (if __valid (setq _valid __valid)) _valid)
  (:heights
   (&optional __heights)
   (if __heights (setq _heights __heights)) _heights)
  (:mean
   (&optional __mean)
   (if __mean (setq _mean __mean)) _mean)
  (:serialization-length
   ()
   (+
    ;; uint8 _status
    1
    ;; bool[] _valid
    (* 1    (length _valid)) 4
    ;; float64[] _heights
    (* 8    (length _heights)) 4
    ;; float64 _mean
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _status
       (write-byte _status s)
     ;; bool[] _valid
     (write-long (length _valid) s)
     (dotimes (i (length _valid))
       (if (elt _valid i) (write-byte -1 s) (write-byte 0 s))
       )
     ;; float64[] _heights
     (write-long (length _heights) s)
     (dotimes (i (length _heights))
       (sys::poke (elt _heights i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64 _mean
       (sys::poke _mean (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _status
     (setq _status (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool[] _valid
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _valid (make-list n))
     (dotimes (i n)
     (setf (elt _valid i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     ))
   ;; float64[] _heights
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _heights (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _heights i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64 _mean
     (setq _mean (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass uav_messages::Height
  :super ros::object
  :slots ())

(setf (get uav_messages::Height :md5sum-) "af6c894dddf3f73449d8d258ec820c83")
(setf (get uav_messages::Height :datatype-) "uav_messages/Height")
(setf (get uav_messages::Height :request) uav_messages::HeightRequest)
(setf (get uav_messages::Height :response) uav_messages::HeightResponse)

(defmethod uav_messages::HeightRequest
  (:response () (instance uav_messages::HeightResponse :init)))

(setf (get uav_messages::HeightRequest :md5sum-) "af6c894dddf3f73449d8d258ec820c83")
(setf (get uav_messages::HeightRequest :datatype-) "uav_messages/HeightRequest")
(setf (get uav_messages::HeightRequest :definition-)
      "uint8 OK=0
uint8 SOME_INVALID=1

geometry_msgs/Vector3[] points

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
---
uint8 status
bool[] valid
float64[] heights
float64 mean
")

(setf (get uav_messages::HeightResponse :md5sum-) "af6c894dddf3f73449d8d258ec820c83")
(setf (get uav_messages::HeightResponse :datatype-) "uav_messages/HeightResponse")
(setf (get uav_messages::HeightResponse :definition-)
      "uint8 OK=0
uint8 SOME_INVALID=1

geometry_msgs/Vector3[] points

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
---
uint8 status
bool[] valid
float64[] heights
float64 mean
")



(provide :uav_messages/Height "af6c894dddf3f73449d8d258ec820c83")


