; Auto-generated. Do not edit!


(cl:in-package point_cloud_transport-msg)


;//! \htmlinclude PointCloudTransportData.msg.html

(cl:defclass <PointCloudTransportData> (roslisp-msg-protocol:ros-message)
  ((string_a
    :reader string_a
    :initarg :string_a
    :type cl:string
    :initform "")
   (number_a
    :reader number_a
    :initarg :number_a
    :type cl:integer
    :initform 0))
)

(cl:defclass PointCloudTransportData (<PointCloudTransportData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PointCloudTransportData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PointCloudTransportData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name point_cloud_transport-msg:<PointCloudTransportData> is deprecated: use point_cloud_transport-msg:PointCloudTransportData instead.")))

(cl:ensure-generic-function 'string_a-val :lambda-list '(m))
(cl:defmethod string_a-val ((m <PointCloudTransportData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader point_cloud_transport-msg:string_a-val is deprecated.  Use point_cloud_transport-msg:string_a instead.")
  (string_a m))

(cl:ensure-generic-function 'number_a-val :lambda-list '(m))
(cl:defmethod number_a-val ((m <PointCloudTransportData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader point_cloud_transport-msg:number_a-val is deprecated.  Use point_cloud_transport-msg:number_a instead.")
  (number_a m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PointCloudTransportData>) ostream)
  "Serializes a message object of type '<PointCloudTransportData>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'string_a))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'string_a))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'number_a)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'number_a)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'number_a)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'number_a)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PointCloudTransportData>) istream)
  "Deserializes a message object of type '<PointCloudTransportData>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'string_a) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'string_a) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'number_a)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'number_a)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'number_a)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'number_a)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PointCloudTransportData>)))
  "Returns string type for a message object of type '<PointCloudTransportData>"
  "point_cloud_transport/PointCloudTransportData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PointCloudTransportData)))
  "Returns string type for a message object of type 'PointCloudTransportData"
  "point_cloud_transport/PointCloudTransportData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PointCloudTransportData>)))
  "Returns md5sum for a message object of type '<PointCloudTransportData>"
  "1041fd5d40af632dd7d721f20580cc73")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PointCloudTransportData)))
  "Returns md5sum for a message object of type 'PointCloudTransportData"
  "1041fd5d40af632dd7d721f20580cc73")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PointCloudTransportData>)))
  "Returns full string definition for message of type '<PointCloudTransportData>"
  (cl:format cl:nil "# TODO: implement message format for transporting compressed point cloud~%~%# placeholder~%~%string string_a~%~%uint32 number_a~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PointCloudTransportData)))
  "Returns full string definition for message of type 'PointCloudTransportData"
  (cl:format cl:nil "# TODO: implement message format for transporting compressed point cloud~%~%# placeholder~%~%string string_a~%~%uint32 number_a~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PointCloudTransportData>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'string_a))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PointCloudTransportData>))
  "Converts a ROS message object to a list"
  (cl:list 'PointCloudTransportData
    (cl:cons ':string_a (string_a msg))
    (cl:cons ':number_a (number_a msg))
))
