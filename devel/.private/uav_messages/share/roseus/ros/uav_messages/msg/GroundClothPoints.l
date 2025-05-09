;; Auto-generated. Do not edit!


(when (boundp 'uav_messages::GroundClothPoints)
  (if (not (find-package "UAV_MESSAGES"))
    (make-package "UAV_MESSAGES"))
  (shadow 'GroundClothPoints (find-package "UAV_MESSAGES")))
(unless (find-package "UAV_MESSAGES::GROUNDCLOTHPOINTS")
  (make-package "UAV_MESSAGES::GROUNDCLOTHPOINTS"))

(in-package "ROS")
;;//! \htmlinclude GroundClothPoints.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass uav_messages::GroundClothPoints
  :super ros::object
  :slots (_origin _width _length _resolution _heights ))

(defmethod uav_messages::GroundClothPoints
  (:init
   (&key
    ((:origin __origin) (instance geometry_msgs::Vector3 :init))
    ((:width __width) 0)
    ((:length __length) 0)
    ((:resolution __resolution) 0.0)
    ((:heights __heights) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _origin __origin)
   (setq _width (round __width))
   (setq _length (round __length))
   (setq _resolution (float __resolution))
   (setq _heights __heights)
   self)
  (:origin
   (&rest __origin)
   (if (keywordp (car __origin))
       (send* _origin __origin)
     (progn
       (if __origin (setq _origin (car __origin)))
       _origin)))
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:length
   (&optional __length)
   (if __length (setq _length __length)) _length)
  (:resolution
   (&optional __resolution)
   (if __resolution (setq _resolution __resolution)) _resolution)
  (:heights
   (&optional __heights)
   (if __heights (setq _heights __heights)) _heights)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Vector3 _origin
    (send _origin :serialization-length)
    ;; uint32 _width
    4
    ;; uint32 _length
    4
    ;; float64 _resolution
    8
    ;; float64[] _heights
    (* 8    (length _heights)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Vector3 _origin
       (send _origin :serialize s)
     ;; uint32 _width
       (write-long _width s)
     ;; uint32 _length
       (write-long _length s)
     ;; float64 _resolution
       (sys::poke _resolution (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64[] _heights
     (write-long (length _heights) s)
     (dotimes (i (length _heights))
       (sys::poke (elt _heights i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Vector3 _origin
     (send _origin :deserialize buf ptr-) (incf ptr- (send _origin :serialization-length))
   ;; uint32 _width
     (setq _width (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _length
     (setq _length (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _resolution
     (setq _resolution (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64[] _heights
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _heights (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _heights i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get uav_messages::GroundClothPoints :md5sum-) "38fbc55ea331f440db17b20331203f67")
(setf (get uav_messages::GroundClothPoints :datatype-) "uav_messages/GroundClothPoints")
(setf (get uav_messages::GroundClothPoints :definition-)
      "# Data alignment origin
geometry_msgs/Vector3 origin
uint32 width
uint32 length
float64 resolution
float64[] heights
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
float64 z
")



(provide :uav_messages/GroundClothPoints "38fbc55ea331f440db17b20331203f67")


