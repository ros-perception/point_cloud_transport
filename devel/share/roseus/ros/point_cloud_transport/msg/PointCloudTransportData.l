;; Auto-generated. Do not edit!


(when (boundp 'point_cloud_transport::PointCloudTransportData)
  (if (not (find-package "POINT_CLOUD_TRANSPORT"))
    (make-package "POINT_CLOUD_TRANSPORT"))
  (shadow 'PointCloudTransportData (find-package "POINT_CLOUD_TRANSPORT")))
(unless (find-package "POINT_CLOUD_TRANSPORT::POINTCLOUDTRANSPORTDATA")
  (make-package "POINT_CLOUD_TRANSPORT::POINTCLOUDTRANSPORTDATA"))

(in-package "ROS")
;;//! \htmlinclude PointCloudTransportData.msg.html


(defclass point_cloud_transport::PointCloudTransportData
  :super ros::object
  :slots (_string_a _number_a ))

(defmethod point_cloud_transport::PointCloudTransportData
  (:init
   (&key
    ((:string_a __string_a) "")
    ((:number_a __number_a) 0)
    )
   (send-super :init)
   (setq _string_a (string __string_a))
   (setq _number_a (round __number_a))
   self)
  (:string_a
   (&optional __string_a)
   (if __string_a (setq _string_a __string_a)) _string_a)
  (:number_a
   (&optional __number_a)
   (if __number_a (setq _number_a __number_a)) _number_a)
  (:serialization-length
   ()
   (+
    ;; string _string_a
    4 (length _string_a)
    ;; uint32 _number_a
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _string_a
       (write-long (length _string_a) s) (princ _string_a s)
     ;; uint32 _number_a
       (write-long _number_a s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _string_a
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _string_a (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint32 _number_a
     (setq _number_a (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get point_cloud_transport::PointCloudTransportData :md5sum-) "1041fd5d40af632dd7d721f20580cc73")
(setf (get point_cloud_transport::PointCloudTransportData :datatype-) "point_cloud_transport/PointCloudTransportData")
(setf (get point_cloud_transport::PointCloudTransportData :definition-)
      "# TODO: implement message format for transporting compressed point cloud

# placeholder

string string_a

uint32 number_a


")



(provide :point_cloud_transport/PointCloudTransportData "1041fd5d40af632dd7d721f20580cc73")


