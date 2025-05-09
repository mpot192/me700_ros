;; Auto-generated. Do not edit!


(when (boundp 'uav_messages::LeftRightRow)
  (if (not (find-package "UAV_MESSAGES"))
    (make-package "UAV_MESSAGES"))
  (shadow 'LeftRightRow (find-package "UAV_MESSAGES")))
(unless (find-package "UAV_MESSAGES::LEFTRIGHTROW")
  (make-package "UAV_MESSAGES::LEFTRIGHTROW"))

(in-package "ROS")
;;//! \htmlinclude LeftRightRow.msg.html


(defclass uav_messages::LeftRightRow
  :super ros::object
  :slots (_left_r _left_theta _right_r _right_theta ))

(defmethod uav_messages::LeftRightRow
  (:init
   (&key
    ((:left_r __left_r) 0.0)
    ((:left_theta __left_theta) 0.0)
    ((:right_r __right_r) 0.0)
    ((:right_theta __right_theta) 0.0)
    )
   (send-super :init)
   (setq _left_r (float __left_r))
   (setq _left_theta (float __left_theta))
   (setq _right_r (float __right_r))
   (setq _right_theta (float __right_theta))
   self)
  (:left_r
   (&optional __left_r)
   (if __left_r (setq _left_r __left_r)) _left_r)
  (:left_theta
   (&optional __left_theta)
   (if __left_theta (setq _left_theta __left_theta)) _left_theta)
  (:right_r
   (&optional __right_r)
   (if __right_r (setq _right_r __right_r)) _right_r)
  (:right_theta
   (&optional __right_theta)
   (if __right_theta (setq _right_theta __right_theta)) _right_theta)
  (:serialization-length
   ()
   (+
    ;; float64 _left_r
    8
    ;; float64 _left_theta
    8
    ;; float64 _right_r
    8
    ;; float64 _right_theta
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _left_r
       (sys::poke _left_r (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _left_theta
       (sys::poke _left_theta (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _right_r
       (sys::poke _right_r (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _right_theta
       (sys::poke _right_theta (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _left_r
     (setq _left_r (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _left_theta
     (setq _left_theta (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _right_r
     (setq _right_r (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _right_theta
     (setq _right_theta (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get uav_messages::LeftRightRow :md5sum-) "9547fcc8a1d9006ddee6af9894e1d3ac")
(setf (get uav_messages::LeftRightRow :datatype-) "uav_messages/LeftRightRow")
(setf (get uav_messages::LeftRightRow :definition-)
      "# Most likely left/right lines in radius/angle formation.
float64 left_r
float64 left_theta
float64 right_r
float64 right_theta
")



(provide :uav_messages/LeftRightRow "9547fcc8a1d9006ddee6af9894e1d3ac")


