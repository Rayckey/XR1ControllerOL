
(cl:in-package :asdf)

(defsystem "xr1controllerol-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "AnimationMsgs" :depends-on ("_package_AnimationMsgs"))
    (:file "_package_AnimationMsgs" :depends-on ("_package"))
    (:file "IKLinearTarget" :depends-on ("_package_IKLinearTarget"))
    (:file "_package_IKLinearTarget" :depends-on ("_package"))
  ))