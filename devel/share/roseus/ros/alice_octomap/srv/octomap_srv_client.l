;; Auto-generated. Do not edit!


(when (boundp 'alice_octomap::octomap_srv_client)
  (if (not (find-package "ALICE_OCTOMAP"))
    (make-package "ALICE_OCTOMAP"))
  (shadow 'octomap_srv_client (find-package "ALICE_OCTOMAP")))
(unless (find-package "ALICE_OCTOMAP::OCTOMAP_SRV_CLIENT")
  (make-package "ALICE_OCTOMAP::OCTOMAP_SRV_CLIENT"))
(unless (find-package "ALICE_OCTOMAP::OCTOMAP_SRV_CLIENTREQUEST")
  (make-package "ALICE_OCTOMAP::OCTOMAP_SRV_CLIENTREQUEST"))
(unless (find-package "ALICE_OCTOMAP::OCTOMAP_SRV_CLIENTRESPONSE")
  (make-package "ALICE_OCTOMAP::OCTOMAP_SRV_CLIENTRESPONSE"))

(in-package "ROS")





(defclass alice_octomap::octomap_srv_clientRequest
  :super ros::object
  :slots ())

(defmethod alice_octomap::octomap_srv_clientRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass alice_octomap::octomap_srv_clientResponse
  :super ros::object
  :slots (_occupied_voxels ))

(defmethod alice_octomap::octomap_srv_clientResponse
  (:init
   (&key
    ((:occupied_voxels __occupied_voxels) 0)
    )
   (send-super :init)
   (setq _occupied_voxels (round __occupied_voxels))
   self)
  (:occupied_voxels
   (&optional __occupied_voxels)
   (if __occupied_voxels (setq _occupied_voxels __occupied_voxels)) _occupied_voxels)
  (:serialization-length
   ()
   (+
    ;; int32 _occupied_voxels
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _occupied_voxels
       (write-long _occupied_voxels s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _occupied_voxels
     (setq _occupied_voxels (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass alice_octomap::octomap_srv_client
  :super ros::object
  :slots ())

(setf (get alice_octomap::octomap_srv_client :md5sum-) "c2ce102c71339eb15c8ed24c4bfa8169")
(setf (get alice_octomap::octomap_srv_client :datatype-) "alice_octomap/octomap_srv_client")
(setf (get alice_octomap::octomap_srv_client :request) alice_octomap::octomap_srv_clientRequest)
(setf (get alice_octomap::octomap_srv_client :response) alice_octomap::octomap_srv_clientResponse)

(defmethod alice_octomap::octomap_srv_clientRequest
  (:response () (instance alice_octomap::octomap_srv_clientResponse :init)))

(setf (get alice_octomap::octomap_srv_clientRequest :md5sum-) "c2ce102c71339eb15c8ed24c4bfa8169")
(setf (get alice_octomap::octomap_srv_clientRequest :datatype-) "alice_octomap/octomap_srv_clientRequest")
(setf (get alice_octomap::octomap_srv_clientRequest :definition-)
      "---
int32 occupied_voxels
")

(setf (get alice_octomap::octomap_srv_clientResponse :md5sum-) "c2ce102c71339eb15c8ed24c4bfa8169")
(setf (get alice_octomap::octomap_srv_clientResponse :datatype-) "alice_octomap/octomap_srv_clientResponse")
(setf (get alice_octomap::octomap_srv_clientResponse :definition-)
      "---
int32 occupied_voxels
")



(provide :alice_octomap/octomap_srv_client "c2ce102c71339eb15c8ed24c4bfa8169")


