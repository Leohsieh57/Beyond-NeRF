; Auto-generated. Do not edit!


(cl:in-package my_listener-msg)


;//! \htmlinclude TimeTransform.msg.html

(cl:defclass <TimeTransform> (roslisp-msg-protocol:ros-message)
  ((geo_trans
    :reader geo_trans
    :initarg :geo_trans
    :type geometry_msgs-msg:Transform
    :initform (cl:make-instance 'geometry_msgs-msg:Transform))
   (t1
    :reader t1
    :initarg :t1
    :type cl:real
    :initform 0)
   (t2
    :reader t2
    :initarg :t2
    :type cl:real
    :initform 0))
)

(cl:defclass TimeTransform (<TimeTransform>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TimeTransform>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TimeTransform)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name my_listener-msg:<TimeTransform> is deprecated: use my_listener-msg:TimeTransform instead.")))

(cl:ensure-generic-function 'geo_trans-val :lambda-list '(m))
(cl:defmethod geo_trans-val ((m <TimeTransform>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_listener-msg:geo_trans-val is deprecated.  Use my_listener-msg:geo_trans instead.")
  (geo_trans m))

(cl:ensure-generic-function 't1-val :lambda-list '(m))
(cl:defmethod t1-val ((m <TimeTransform>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_listener-msg:t1-val is deprecated.  Use my_listener-msg:t1 instead.")
  (t1 m))

(cl:ensure-generic-function 't2-val :lambda-list '(m))
(cl:defmethod t2-val ((m <TimeTransform>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_listener-msg:t2-val is deprecated.  Use my_listener-msg:t2 instead.")
  (t2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TimeTransform>) ostream)
  "Serializes a message object of type '<TimeTransform>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'geo_trans) ostream)
  (cl:let ((__sec (cl:floor (cl:slot-value msg 't1)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 't1) (cl:floor (cl:slot-value msg 't1)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:let ((__sec (cl:floor (cl:slot-value msg 't2)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 't2) (cl:floor (cl:slot-value msg 't2)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TimeTransform>) istream)
  "Deserializes a message object of type '<TimeTransform>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'geo_trans) istream)
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 't1) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 't2) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TimeTransform>)))
  "Returns string type for a message object of type '<TimeTransform>"
  "my_listener/TimeTransform")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TimeTransform)))
  "Returns string type for a message object of type 'TimeTransform"
  "my_listener/TimeTransform")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TimeTransform>)))
  "Returns md5sum for a message object of type '<TimeTransform>"
  "b05693e338e8c00beb115d4348be07e1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TimeTransform)))
  "Returns md5sum for a message object of type 'TimeTransform"
  "b05693e338e8c00beb115d4348be07e1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TimeTransform>)))
  "Returns full string definition for message of type '<TimeTransform>"
  (cl:format cl:nil "geometry_msgs/Transform geo_trans~%time t1~%time t2~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TimeTransform)))
  "Returns full string definition for message of type 'TimeTransform"
  (cl:format cl:nil "geometry_msgs/Transform geo_trans~%time t1~%time t2~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TimeTransform>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'geo_trans))
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TimeTransform>))
  "Converts a ROS message object to a list"
  (cl:list 'TimeTransform
    (cl:cons ':geo_trans (geo_trans msg))
    (cl:cons ':t1 (t1 msg))
    (cl:cons ':t2 (t2 msg))
))
