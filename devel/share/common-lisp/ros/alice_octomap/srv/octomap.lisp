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
  "d6c656edb646bcaf655c3aa23e2812b3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'octomap-request)))
  "Returns md5sum for a message object of type 'octomap-request"
  "d6c656edb646bcaf655c3aa23e2812b3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<octomap-request>)))
  "Returns full string definition for message of type '<octomap-request>"
  (cl:format cl:nil "#Service receives no input~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'octomap-request)))
  "Returns full string definition for message of type 'octomap-request"
  (cl:format cl:nil "#Service receives no input~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <octomap-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <octomap-request>))
  "Converts a ROS message object to a list"
  (cl:list 'octomap-request
))
;//! \htmlinclude octomap-response.msg.html

(cl:defclass <octomap-response> (roslisp-msg-protocol:ros-message)
  ((occupied_voxels
    :reader occupied_voxels
    :initarg :occupied_voxels
    :type cl:integer
    :initform 0)
   (total_voxels
    :reader total_voxels
    :initarg :total_voxels
    :type cl:integer
    :initform 0))
)

(cl:defclass octomap-response (<octomap-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <octomap-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'octomap-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name alice_octomap-srv:<octomap-response> is deprecated: use alice_octomap-srv:octomap-response instead.")))

(cl:ensure-generic-function 'occupied_voxels-val :lambda-list '(m))
(cl:defmethod occupied_voxels-val ((m <octomap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alice_octomap-srv:occupied_voxels-val is deprecated.  Use alice_octomap-srv:occupied_voxels instead.")
  (occupied_voxels m))

(cl:ensure-generic-function 'total_voxels-val :lambda-list '(m))
(cl:defmethod total_voxels-val ((m <octomap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alice_octomap-srv:total_voxels-val is deprecated.  Use alice_octomap-srv:total_voxels instead.")
  (total_voxels m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <octomap-response>) ostream)
  "Serializes a message object of type '<octomap-response>"
  (cl:let* ((signed (cl:slot-value msg 'occupied_voxels)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'total_voxels)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <octomap-response>) istream)
  "Deserializes a message object of type '<octomap-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'occupied_voxels) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'total_voxels) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
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
  "d6c656edb646bcaf655c3aa23e2812b3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'octomap-response)))
  "Returns md5sum for a message object of type 'octomap-response"
  "d6c656edb646bcaf655c3aa23e2812b3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<octomap-response>)))
  "Returns full string definition for message of type '<octomap-response>"
  (cl:format cl:nil "int32 occupied_voxels~%int32 total_voxels~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'octomap-response)))
  "Returns full string definition for message of type 'octomap-response"
  (cl:format cl:nil "int32 occupied_voxels~%int32 total_voxels~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <octomap-response>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <octomap-response>))
  "Converts a ROS message object to a list"
  (cl:list 'octomap-response
    (cl:cons ':occupied_voxels (occupied_voxels msg))
    (cl:cons ':total_voxels (total_voxels msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'octomap)))
  'octomap-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'octomap)))
  'octomap-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'octomap)))
  "Returns string type for a service object of type '<octomap>"
  "alice_octomap/octomap")