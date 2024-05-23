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
  :slots (_total_voxels _occupied_voxels _x_values _y_values _z_values _occupancy ))

(defmethod alice_octomap::octomapResponse
  (:init
   (&key
    ((:total_voxels __total_voxels) 0)
    ((:occupied_voxels __occupied_voxels) 0)
    ((:x_values __x_values) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:y_values __y_values) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:z_values __z_values) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:occupancy __occupancy) (let (r) (dotimes (i 0) (push nil r)) r))
    )
   (send-super :init)
   (setq _total_voxels (round __total_voxels))
   (setq _occupied_voxels (round __occupied_voxels))
   (setq _x_values __x_values)
   (setq _y_values __y_values)
   (setq _z_values __z_values)
   (setq _occupancy __occupancy)
   self)
  (:total_voxels
   (&optional __total_voxels)
   (if __total_voxels (setq _total_voxels __total_voxels)) _total_voxels)
  (:occupied_voxels
   (&optional __occupied_voxels)
   (if __occupied_voxels (setq _occupied_voxels __occupied_voxels)) _occupied_voxels)
  (:x_values
   (&optional __x_values)
   (if __x_values (setq _x_values __x_values)) _x_values)
  (:y_values
   (&optional __y_values)
   (if __y_values (setq _y_values __y_values)) _y_values)
  (:z_values
   (&optional __z_values)
   (if __z_values (setq _z_values __z_values)) _z_values)
  (:occupancy
   (&optional __occupancy)
   (if __occupancy (setq _occupancy __occupancy)) _occupancy)
  (:serialization-length
   ()
   (+
    ;; int32 _total_voxels
    4
    ;; int32 _occupied_voxels
    4
    ;; float64[] _x_values
    (* 8    (length _x_values)) 4
    ;; float64[] _y_values
    (* 8    (length _y_values)) 4
    ;; float64[] _z_values
    (* 8    (length _z_values)) 4
    ;; bool[] _occupancy
    (* 1    (length _occupancy)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _total_voxels
       (write-long _total_voxels s)
     ;; int32 _occupied_voxels
       (write-long _occupied_voxels s)
     ;; float64[] _x_values
     (write-long (length _x_values) s)
     (dotimes (i (length _x_values))
       (sys::poke (elt _x_values i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _y_values
     (write-long (length _y_values) s)
     (dotimes (i (length _y_values))
       (sys::poke (elt _y_values i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _z_values
     (write-long (length _z_values) s)
     (dotimes (i (length _z_values))
       (sys::poke (elt _z_values i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; bool[] _occupancy
     (write-long (length _occupancy) s)
     (dotimes (i (length _occupancy))
       (if (elt _occupancy i) (write-byte -1 s) (write-byte 0 s))
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _total_voxels
     (setq _total_voxels (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _occupied_voxels
     (setq _occupied_voxels (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64[] _x_values
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _x_values (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _x_values i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _y_values
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _y_values (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _y_values i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _z_values
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _z_values (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _z_values i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; bool[] _occupancy
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _occupancy (make-list n))
     (dotimes (i n)
     (setf (elt _occupancy i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     ))
   ;;
   self)
  )

(defclass alice_octomap::octomap
  :super ros::object
  :slots ())

(setf (get alice_octomap::octomap :md5sum-) "e49c62c1add6ce5ed13e188a48e66fc4")
(setf (get alice_octomap::octomap :datatype-) "alice_octomap/octomap")
(setf (get alice_octomap::octomap :request) alice_octomap::octomapRequest)
(setf (get alice_octomap::octomap :response) alice_octomap::octomapResponse)

(defmethod alice_octomap::octomapRequest
  (:response () (instance alice_octomap::octomapResponse :init)))

(setf (get alice_octomap::octomapRequest :md5sum-) "e49c62c1add6ce5ed13e188a48e66fc4")
(setf (get alice_octomap::octomapRequest :datatype-) "alice_octomap/octomapRequest")
(setf (get alice_octomap::octomapRequest :definition-)
      "---
int32 total_voxels
int32 occupied_voxels
float64[] x_values
float64[] y_values
float64[] z_values
bool[] occupancy
")

(setf (get alice_octomap::octomapResponse :md5sum-) "e49c62c1add6ce5ed13e188a48e66fc4")
(setf (get alice_octomap::octomapResponse :datatype-) "alice_octomap/octomapResponse")
(setf (get alice_octomap::octomapResponse :definition-)
      "---
int32 total_voxels
int32 occupied_voxels
float64[] x_values
float64[] y_values
float64[] z_values
bool[] occupancy
")



(provide :alice_octomap/octomap "e49c62c1add6ce5ed13e188a48e66fc4")


