
(cl:in-package :asdf)

(defsystem "yolov8qt-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "RJ6KData" :depends-on ("_package_RJ6KData"))
    (:file "_package_RJ6KData" :depends-on ("_package"))
  ))