;; Auto-generated. Do not edit!


(when (boundp 'ti_mmwave_rospkg::mmWaveCLI)
  (if (not (find-package "TI_MMWAVE_ROSPKG"))
    (make-package "TI_MMWAVE_ROSPKG"))
  (shadow 'mmWaveCLI (find-package "TI_MMWAVE_ROSPKG")))
(unless (find-package "TI_MMWAVE_ROSPKG::MMWAVECLI")
  (make-package "TI_MMWAVE_ROSPKG::MMWAVECLI"))
(unless (find-package "TI_MMWAVE_ROSPKG::MMWAVECLIREQUEST")
  (make-package "TI_MMWAVE_ROSPKG::MMWAVECLIREQUEST"))
(unless (find-package "TI_MMWAVE_ROSPKG::MMWAVECLIRESPONSE")
  (make-package "TI_MMWAVE_ROSPKG::MMWAVECLIRESPONSE"))

(in-package "ROS")





(defclass ti_mmwave_rospkg::mmWaveCLIRequest
  :super ros::object
  :slots (_comm ))

(defmethod ti_mmwave_rospkg::mmWaveCLIRequest
  (:init
   (&key
    ((:comm __comm) "")
    )
   (send-super :init)
   (setq _comm (string __comm))
   self)
  (:comm
   (&optional __comm)
   (if __comm (setq _comm __comm)) _comm)
  (:serialization-length
   ()
   (+
    ;; string _comm
    4 (length _comm)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _comm
       (write-long (length _comm) s) (princ _comm s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _comm
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _comm (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass ti_mmwave_rospkg::mmWaveCLIResponse
  :super ros::object
  :slots (_resp ))

(defmethod ti_mmwave_rospkg::mmWaveCLIResponse
  (:init
   (&key
    ((:resp __resp) "")
    )
   (send-super :init)
   (setq _resp (string __resp))
   self)
  (:resp
   (&optional __resp)
   (if __resp (setq _resp __resp)) _resp)
  (:serialization-length
   ()
   (+
    ;; string _resp
    4 (length _resp)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _resp
       (write-long (length _resp) s) (princ _resp s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _resp
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _resp (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass ti_mmwave_rospkg::mmWaveCLI
  :super ros::object
  :slots ())

(setf (get ti_mmwave_rospkg::mmWaveCLI :md5sum-) "f079c47a57c95983638c539cc506d12d")
(setf (get ti_mmwave_rospkg::mmWaveCLI :datatype-) "ti_mmwave_rospkg/mmWaveCLI")
(setf (get ti_mmwave_rospkg::mmWaveCLI :request) ti_mmwave_rospkg::mmWaveCLIRequest)
(setf (get ti_mmwave_rospkg::mmWaveCLI :response) ti_mmwave_rospkg::mmWaveCLIResponse)

(defmethod ti_mmwave_rospkg::mmWaveCLIRequest
  (:response () (instance ti_mmwave_rospkg::mmWaveCLIResponse :init)))

(setf (get ti_mmwave_rospkg::mmWaveCLIRequest :md5sum-) "f079c47a57c95983638c539cc506d12d")
(setf (get ti_mmwave_rospkg::mmWaveCLIRequest :datatype-) "ti_mmwave_rospkg/mmWaveCLIRequest")
(setf (get ti_mmwave_rospkg::mmWaveCLIRequest :definition-)
      "string comm
---
string resp

")

(setf (get ti_mmwave_rospkg::mmWaveCLIResponse :md5sum-) "f079c47a57c95983638c539cc506d12d")
(setf (get ti_mmwave_rospkg::mmWaveCLIResponse :datatype-) "ti_mmwave_rospkg/mmWaveCLIResponse")
(setf (get ti_mmwave_rospkg::mmWaveCLIResponse :definition-)
      "string comm
---
string resp

")



(provide :ti_mmwave_rospkg/mmWaveCLI "f079c47a57c95983638c539cc506d12d")


