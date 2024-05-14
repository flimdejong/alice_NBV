;; Auto-generated. Do not edit!


(when (boundp 'alice_octomap::octomap)
  (if (not (find-package "ALICE_OCTOMAP"))
    (make-package "ALICE_OCTOMAP"))
  (shadow 'octomap (find-package "ALICE_OCTOMAP")))
(unless (find-package "ALICE_OCTOMAP::OCTOMAP")
  (make-package "ALICE_OCTOMAP::OCTOMAP"))
(unless (find-package "ALICE_OCTOMAP::OCTOMAPREQUEST")
  (make-package "ALICE_OCTOMAP::OCTOMAPREQUEST"))
(unless (find-package "ALICE_OCTOMAP::OCTOMAPRESPONSE")
  (make-package "ALICE_OCTOMAP::OCTOMAPRESPONSE"))

(in-package "ROS")





(defclass alice_octomap::octomapRequest
  :super ros::object
  :slots ())

(defmethod alice_octomap::octomapRequest
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

(defclass alice_octomap::octomapResponse
  :super ros::object
  :slots (_occupied_voxels _total_voxels ))

(defmethod alice_octomap::octomapResponse
  (:init
   (&key
    ((:occupied_voxels __occupied_voxels) 0)
    ((:total_voxels __total_voxels) 0)
    )
   (send-super :init)
   (setq _occupied_voxels (round __occupied_voxels))
   (setq _total_voxels (round __total_voxels))
   self)
  (:occupied_voxels
   (&optional __occupied_voxels)
   (if __occupied_voxels (setq _occupied_voxels __occupied_voxels)) _occupied_voxels)
  (:total_voxels
   (&optional __total_voxels)
   (if __total_voxels (setq _total_voxels __total_voxels)) _total_voxels)
  (:serialization-length
   ()
   (+
    ;; int32 _occupied_voxels
    4
    ;; int32 _total_voxels
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _occupied_voxels
       (write-long _occupied_voxels s)
     ;; int32 _total_voxels
       (write-long _total_voxels s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _occupied_voxels
     (setq _occupied_voxels (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _total_voxels
     (setq _total_voxels (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass alice_octomap::octomap
  :super ros::object
  :slots ())

(setf (get alice_octomap::octomap :md5sum-) "d6c656edb646bcaf655c3aa23e2812b3")
(setf (get alice_octomap::octomap :datatype-) "alice_octomap/octomap")
(setf (get alice_octomap::octomap :request) alice_octomap::octomapRequest)
(setf (get alice_octomap::octomap :response) alice_octomap::octomapResponse)

(defmethod alice_octomap::octomapRequest
  (:response () (instance alice_octomap::octomapResponse :init)))

(setf (get alice_octomap::octomapRequest :md5sum-) "d6c656edb646bcaf655c3aa23e2812b3")
(setf (get alice_octomap::octomapRequest :datatype-) "alice_octomap/octomapRequest")
(setf (get alice_octomap::octomapRequest :definition-)
      "#Service receives no input

---
int32 occupied_voxels
int32 total_voxels
")

(setf (get alice_octomap::octomapResponse :md5sum-) "d6c656edb646bcaf655c3aa23e2812b3")
(setf (get alice_octomap::octomapResponse :datatype-) "alice_octomap/octomapResponse")
(setf (get alice_octomap::octomapResponse :definition-)
      "#Service receives no input

---
int32 occupied_voxels
int32 total_voxels
")



(provide :alice_octomap/octomap "d6c656edb646bcaf655c3aa23e2812b3")


