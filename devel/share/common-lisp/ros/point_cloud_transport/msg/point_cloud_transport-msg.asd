
(cl:in-package :asdf)

(defsystem "point_cloud_transport-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "PointCloudTransportData" :depends-on ("_package_PointCloudTransportData"))
    (:file "_package_PointCloudTransportData" :depends-on ("_package"))
  ))