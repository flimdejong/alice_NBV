; Auto-generated. Do not edit!


(cl:in-package alice_octomap-srv)


;//! \htmlinclude octomap-request.msg.html

(cl:defclass <octomap-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass octomap-request (<octomap-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <octomap-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'octomap-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name alice_octomap-srv:<octomap-request> is deprecated: use alice_octomap-srv:octomap-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <octomap-request>) ostream)
  "Serializes a message object of type '<octomap-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <octomap-request>) istream)
  "Deserializes a message object of type '<octomap-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<octomap-request>)))
  "Returns string type for a service object of type '<octomap-request>"
  "alice_octomap/octomapRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'octomap-request)))
  "Returns string type for a service object of type 'octomap-request"
  "alice_octomap/octomapRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<octomap-request>)))
  "Returns md5sum for a message object of type '<octomap-request>"
  "e49c62c1add6ce5ed13e188a48e66fc4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'octomap-request)))
  "Returns md5sum for a message object of type 'octomap-request"
  "e49c62c1add6ce5ed13e188a48e66fc4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<octomap-request>)))
  "Returns full string definition for message of type '<octomap-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'octomap-request)))
  "Returns full string definition for message of type 'octomap-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <octomap-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <octomap-request>))
  "Converts a ROS message object to a list"
  (cl:list 'octomap-request
))
;//! \htmlinclude octomap-response.msg.html

(cl:defclass <octomap-response> (roslisp-msg-protocol:ros-message)
  ((total_voxels
    :reader total_voxels
    :initarg :total_voxels
    :type cl:integer
    :initform 0)
   (occupied_voxels
    :reader occupied_voxels
    :initarg :occupied_voxels
    :type cl:integer
    :initform 0)
   (x_values
    :reader x_values
    :initarg :x_values
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (y_values
    :reader y_values
    :initarg :y_values
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (z_values
    :reader z_values
    :initarg :z_values
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (occupancy
    :reader occupancy
    :initarg :occupancy
    :type (cl:vector cl:boolean)
   :initform (cl:make-array 0 :element-type 'cl:boolean :initial-element cl:nil)))
)

(cl:defclass octomap-response (<octomap-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <octomap-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'octomap-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name alice_octomap-srv:<octomap-response> is deprecated: use alice_octomap-srv:octomap-response instead.")))

(cl:ensure-generic-function 'total_voxels-val :lambda-list '(m))
(cl:defmethod total_voxels-val ((m <octomap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alice_octomap-srv:total_voxels-val is deprecated.  Use alice_octomap-srv:total_voxels instead.")
  (total_voxels m))

(cl:ensure-generic-function 'occupied_voxels-val :lambda-list '(m))
(cl:defmethod occupied_voxels-val ((m <octomap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alice_octomap-srv:occupied_voxels-val is deprecated.  Use alice_octomap-srv:occupied_voxels instead.")
  (occupied_voxels m))

(cl:ensure-generic-function 'x_values-val :lambda-list '(m))
(cl:defmethod x_values-val ((m <octomap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alice_octomap-srv:x_values-val is deprecated.  Use alice_octomap-srv:x_values instead.")
  (x_values m))

(cl:ensure-generic-function 'y_values-val :lambda-list '(m))
(cl:defmethod y_values-val ((m <octomap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alice_octomap-srv:y_values-val is deprecated.  Use alice_octomap-srv:y_values instead.")
  (y_values m))

(cl:ensure-generic-function 'z_values-val :lambda-list '(m))
(cl:defmethod z_values-val ((m <octomap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alice_octomap-srv:z_values-val is deprecated.  Use alice_octomap-srv:z_values instead.")
  (z_values m))

(cl:ensure-generic-function 'occupancy-val :lambda-list '(m))
(cl:defmethod occupancy-val ((m <octomap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alice_octomap-srv:occupancy-val is deprecated.  Use alice_octomap-srv:occupancy instead.")
  (occupancy m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <octomap-response>) ostream)
  "Serializes a message object of type '<octomap-response>"
  (cl:let* ((signed (cl:slot-value msg 'total_voxels)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'occupied_voxels)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'x_values))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'x_values))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'y_values))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'y_values))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'z_values))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'z_values))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'occupancy))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if ele 1 0)) ostream))
   (cl:slot-value msg 'occupancy))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <octomap-response>) istream)
  "Deserializes a message object of type '<octomap-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'total_voxels) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'occupied_voxels) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'x_values) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'x_values)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'y_values) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'y_values)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'z_values) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'z_values)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'occupancy) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'occupancy)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:not (cl:zerop (cl:read-byte istream)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<octomap-response>)))
  "Returns string type for a service object of type '<octomap-response>"
  "alice_octomap/octomapResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'octomap-response)))
  "Returns string type for a service object of type 'octomap-response"
  "alice_octomap/octomapResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<octomap-response>)))
  "Returns md5sum for a message object of type '<octomap-response>"
  "e49c62c1add6ce5ed13e188a48e66fc4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'octomap-response)))
  "Returns md5sum for a message object of type 'octomap-response"
  "e49c62c1add6ce5ed13e188a48e66fc4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<octomap-response>)))
  "Returns full string definition for message of type '<octomap-response>"
  (cl:format cl:nil "int32 total_voxels~%int32 occupied_voxels~%float64[] x_values~%float64[] y_values~%float64[] z_values~%bool[] occupancy~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'octomap-response)))
  "Returns full string definition for message of type 'octomap-response"
  (cl:format cl:nil "int32 total_voxels~%int32 occupied_voxels~%float64[] x_values~%float64[] y_values~%float64[] z_values~%bool[] occupancy~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <octomap-response>))
  (cl:+ 0
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'x_values) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'y_values) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'z_values) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'occupancy) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <octomap-response>))
  "Converts a ROS message object to a list"
  (cl:list 'octomap-response
    (cl:cons ':total_voxels (total_voxels msg))
    (cl:cons ':occupied_voxels (occupied_voxels msg))
    (cl:cons ':x_values (x_values msg))
    (cl:cons ':y_values (y_values msg))
    (cl:cons ':z_values (z_values msg))
    (cl:cons ':occupancy (occupancy msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'octomap)))
  'octomap-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'octomap)))
  'octomap-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'octomap)))
  "Returns string type for a service object of type '<octomap>"
  "alice_octomap/octomap")