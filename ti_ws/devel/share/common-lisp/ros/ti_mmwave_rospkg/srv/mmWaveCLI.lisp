; Auto-generated. Do not edit!


(cl:in-package ti_mmwave_rospkg-srv)


;//! \htmlinclude mmWaveCLI-request.msg.html

(cl:defclass <mmWaveCLI-request> (roslisp-msg-protocol:ros-message)
  ((comm
    :reader comm
    :initarg :comm
    :type cl:string
    :initform ""))
)

(cl:defclass mmWaveCLI-request (<mmWaveCLI-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <mmWaveCLI-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'mmWaveCLI-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ti_mmwave_rospkg-srv:<mmWaveCLI-request> is deprecated: use ti_mmwave_rospkg-srv:mmWaveCLI-request instead.")))

(cl:ensure-generic-function 'comm-val :lambda-list '(m))
(cl:defmethod comm-val ((m <mmWaveCLI-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ti_mmwave_rospkg-srv:comm-val is deprecated.  Use ti_mmwave_rospkg-srv:comm instead.")
  (comm m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <mmWaveCLI-request>) ostream)
  "Serializes a message object of type '<mmWaveCLI-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'comm))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'comm))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <mmWaveCLI-request>) istream)
  "Deserializes a message object of type '<mmWaveCLI-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'comm) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'comm) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<mmWaveCLI-request>)))
  "Returns string type for a service object of type '<mmWaveCLI-request>"
  "ti_mmwave_rospkg/mmWaveCLIRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'mmWaveCLI-request)))
  "Returns string type for a service object of type 'mmWaveCLI-request"
  "ti_mmwave_rospkg/mmWaveCLIRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<mmWaveCLI-request>)))
  "Returns md5sum for a message object of type '<mmWaveCLI-request>"
  "f079c47a57c95983638c539cc506d12d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'mmWaveCLI-request)))
  "Returns md5sum for a message object of type 'mmWaveCLI-request"
  "f079c47a57c95983638c539cc506d12d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<mmWaveCLI-request>)))
  "Returns full string definition for message of type '<mmWaveCLI-request>"
  (cl:format cl:nil "string comm~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'mmWaveCLI-request)))
  "Returns full string definition for message of type 'mmWaveCLI-request"
  (cl:format cl:nil "string comm~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <mmWaveCLI-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'comm))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <mmWaveCLI-request>))
  "Converts a ROS message object to a list"
  (cl:list 'mmWaveCLI-request
    (cl:cons ':comm (comm msg))
))
;//! \htmlinclude mmWaveCLI-response.msg.html

(cl:defclass <mmWaveCLI-response> (roslisp-msg-protocol:ros-message)
  ((resp
    :reader resp
    :initarg :resp
    :type cl:string
    :initform ""))
)

(cl:defclass mmWaveCLI-response (<mmWaveCLI-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <mmWaveCLI-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'mmWaveCLI-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ti_mmwave_rospkg-srv:<mmWaveCLI-response> is deprecated: use ti_mmwave_rospkg-srv:mmWaveCLI-response instead.")))

(cl:ensure-generic-function 'resp-val :lambda-list '(m))
(cl:defmethod resp-val ((m <mmWaveCLI-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ti_mmwave_rospkg-srv:resp-val is deprecated.  Use ti_mmwave_rospkg-srv:resp instead.")
  (resp m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <mmWaveCLI-response>) ostream)
  "Serializes a message object of type '<mmWaveCLI-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'resp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'resp))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <mmWaveCLI-response>) istream)
  "Deserializes a message object of type '<mmWaveCLI-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'resp) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'resp) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<mmWaveCLI-response>)))
  "Returns string type for a service object of type '<mmWaveCLI-response>"
  "ti_mmwave_rospkg/mmWaveCLIResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'mmWaveCLI-response)))
  "Returns string type for a service object of type 'mmWaveCLI-response"
  "ti_mmwave_rospkg/mmWaveCLIResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<mmWaveCLI-response>)))
  "Returns md5sum for a message object of type '<mmWaveCLI-response>"
  "f079c47a57c95983638c539cc506d12d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'mmWaveCLI-response)))
  "Returns md5sum for a message object of type 'mmWaveCLI-response"
  "f079c47a57c95983638c539cc506d12d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<mmWaveCLI-response>)))
  "Returns full string definition for message of type '<mmWaveCLI-response>"
  (cl:format cl:nil "string resp~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'mmWaveCLI-response)))
  "Returns full string definition for message of type 'mmWaveCLI-response"
  (cl:format cl:nil "string resp~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <mmWaveCLI-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'resp))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <mmWaveCLI-response>))
  "Converts a ROS message object to a list"
  (cl:list 'mmWaveCLI-response
    (cl:cons ':resp (resp msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'mmWaveCLI)))
  'mmWaveCLI-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'mmWaveCLI)))
  'mmWaveCLI-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'mmWaveCLI)))
  "Returns string type for a service object of type '<mmWaveCLI>"
  "ti_mmwave_rospkg/mmWaveCLI")