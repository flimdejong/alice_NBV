; Auto-generated. Do not edit!


(cl:in-package alice_octomap-srv)


;//! \htmlinclude octomap_srv_client-request.msg.html

(cl:defclass <octomap_srv_client-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass octomap_srv_client-request (<octomap_srv_client-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <octomap_srv_client-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'octomap_srv_client-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name alice_octomap-srv:<octomap_srv_client-request> is deprecated: use alice_octomap-srv:octomap_srv_client-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <octomap_srv_client-request>) ostream)
  "Serializes a message object of type '<octomap_srv_client-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <octomap_srv_client-request>) istream)
  "Deserializes a message object of type '<octomap_srv_client-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<octomap_srv_client-request>)))
  "Returns string type for a service object of type '<octomap_srv_client-request>"
  "alice_octomap/octomap_srv_clientRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'octomap_srv_client-request)))
  "Returns string type for a service object of type 'octomap_srv_client-request"
  "alice_octomap/octomap_srv_clientRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<octomap_srv_client-request>)))
  "Returns md5sum for a message object of type '<octomap_srv_client-request>"
  "c2ce102c71339eb15c8ed24c4bfa8169")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'octomap_srv_client-request)))
  "Returns md5sum for a message object of type 'octomap_srv_client-request"
  "c2ce102c71339eb15c8ed24c4bfa8169")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<octomap_srv_client-request>)))
  "Returns full string definition for message of type '<octomap_srv_client-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'octomap_srv_client-request)))
  "Returns full string definition for message of type 'octomap_srv_client-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <octomap_srv_client-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <octomap_srv_client-request>))
  "Converts a ROS message object to a list"
  (cl:list 'octomap_srv_client-request
))
;//! \htmlinclude octomap_srv_client-response.msg.html

(cl:defclass <octomap_srv_client-response> (roslisp-msg-protocol:ros-message)
  ((occupied_voxels
    :reader occupied_voxels
    :initarg :occupied_voxels
    :type cl:integer
    :initform 0))
)

(cl:defclass octomap_srv_client-response (<octomap_srv_client-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <octomap_srv_client-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'octomap_srv_client-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name alice_octomap-srv:<octomap_srv_client-response> is deprecated: use alice_octomap-srv:octomap_srv_client-response instead.")))

(cl:ensure-generic-function 'occupied_voxels-val :lambda-list '(m))
(cl:defmethod occupied_voxels-val ((m <octomap_srv_client-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alice_octomap-srv:occupied_voxels-val is deprecated.  Use alice_octomap-srv:occupied_voxels instead.")
  (occupied_voxels m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <octomap_srv_client-response>) ostream)
  "Serializes a message object of type '<octomap_srv_client-response>"
  (cl:let* ((signed (cl:slot-value msg 'occupied_voxels)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <octomap_srv_client-response>) istream)
  "Deserializes a message object of type '<octomap_srv_client-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'occupied_voxels) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<octomap_srv_client-response>)))
  "Returns string type for a service object of type '<octomap_srv_client-response>"
  "alice_octomap/octomap_srv_clientResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'octomap_srv_client-response)))
  "Returns string type for a service object of type 'octomap_srv_client-response"
  "alice_octomap/octomap_srv_clientResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<octomap_srv_client-response>)))
  "Returns md5sum for a message object of type '<octomap_srv_client-response>"
  "c2ce102c71339eb15c8ed24c4bfa8169")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'octomap_srv_client-response)))
  "Returns md5sum for a message object of type 'octomap_srv_client-response"
  "c2ce102c71339eb15c8ed24c4bfa8169")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<octomap_srv_client-response>)))
  "Returns full string definition for message of type '<octomap_srv_client-response>"
  (cl:format cl:nil "int32 occupied_voxels~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'octomap_srv_client-response)))
  "Returns full string definition for message of type 'octomap_srv_client-response"
  (cl:format cl:nil "int32 occupied_voxels~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <octomap_srv_client-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <octomap_srv_client-response>))
  "Converts a ROS message object to a list"
  (cl:list 'octomap_srv_client-response
    (cl:cons ':occupied_voxels (occupied_voxels msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'octomap_srv_client)))
  'octomap_srv_client-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'octomap_srv_client)))
  'octomap_srv_client-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'octomap_srv_client)))
  "Returns string type for a service object of type '<octomap_srv_client>"
  "alice_octomap/octomap_srv_client")