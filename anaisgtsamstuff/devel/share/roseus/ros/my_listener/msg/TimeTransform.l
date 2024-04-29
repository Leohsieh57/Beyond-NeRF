;; Auto-generated. Do not edit!


(when (boundp 'my_listener::TimeTransform)
  (if (not (find-package "MY_LISTENER"))
    (make-package "MY_LISTENER"))
  (shadow 'TimeTransform (find-package "MY_LISTENER")))
(unless (find-package "MY_LISTENER::TIMETRANSFORM")
  (make-package "MY_LISTENER::TIMETRANSFORM"))

(in-package "ROS")
;;//! \htmlinclude TimeTransform.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass my_listener::TimeTransform
  :super ros::object
  :slots (_geo_trans _t1 _t2 ))

(defmethod my_listener::TimeTransform
  (:init
   (&key
    ((:geo_trans __geo_trans) (instance geometry_msgs::Transform :init))
    ((:t1 __t1) (instance ros::time :init))
    ((:t2 __t2) (instance ros::time :init))
    )
   (send-super :init)
   (setq _geo_trans __geo_trans)
   (setq _t1 __t1)
   (setq _t2 __t2)
   self)
  (:geo_trans
   (&rest __geo_trans)
   (if (keywordp (car __geo_trans))
       (send* _geo_trans __geo_trans)
     (progn
       (if __geo_trans (setq _geo_trans (car __geo_trans)))
       _geo_trans)))
  (:t1
   (&optional __t1)
   (if __t1 (setq _t1 __t1)) _t1)
  (:t2
   (&optional __t2)
   (if __t2 (setq _t2 __t2)) _t2)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Transform _geo_trans
    (send _geo_trans :serialization-length)
    ;; time _t1
    8
    ;; time _t2
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Transform _geo_trans
       (send _geo_trans :serialize s)
     ;; time _t1
       (write-long (send _t1 :sec) s) (write-long (send _t1 :nsec) s)
     ;; time _t2
       (write-long (send _t2 :sec) s) (write-long (send _t2 :nsec) s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Transform _geo_trans
     (send _geo_trans :deserialize buf ptr-) (incf ptr- (send _geo_trans :serialization-length))
   ;; time _t1
     (send _t1 :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _t1 :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; time _t2
     (send _t2 :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _t2 :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get my_listener::TimeTransform :md5sum-) "b05693e338e8c00beb115d4348be07e1")
(setf (get my_listener::TimeTransform :datatype-) "my_listener/TimeTransform")
(setf (get my_listener::TimeTransform :definition-)
      "geometry_msgs/Transform geo_trans
time t1
time t2

================================================================================
MSG: geometry_msgs/Transform
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation

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
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :my_listener/TimeTransform "b05693e338e8c00beb115d4348be07e1")


