
(cl:in-package :asdf)

(defsystem "ti_mmwave_rospkg-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "mmWaveCLI" :depends-on ("_package_mmWaveCLI"))
    (:file "_package_mmWaveCLI" :depends-on ("_package"))
  ))