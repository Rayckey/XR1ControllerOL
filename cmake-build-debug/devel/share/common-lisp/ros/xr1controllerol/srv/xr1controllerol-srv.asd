
(cl:in-package :asdf)

(defsystem "xr1controllerol-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "AnimationOverwrite" :depends-on ("_package_AnimationOverwrite"))
    (:file "_package_AnimationOverwrite" :depends-on ("_package"))
    (:file "AnimationQuery" :depends-on ("_package_AnimationQuery"))
    (:file "_package_AnimationQuery" :depends-on ("_package"))
    (:file "BalanceQuery" :depends-on ("_package_BalanceQuery"))
    (:file "_package_BalanceQuery" :depends-on ("_package"))
    (:file "HandGripQuery" :depends-on ("_package_HandGripQuery"))
    (:file "_package_HandGripQuery" :depends-on ("_package"))
    (:file "IKLinearService" :depends-on ("_package_IKLinearService"))
    (:file "_package_IKLinearService" :depends-on ("_package"))
    (:file "IKPlannerService" :depends-on ("_package_IKPlannerService"))
    (:file "_package_IKPlannerService" :depends-on ("_package"))
    (:file "IKTrackingService" :depends-on ("_package_IKTrackingService"))
    (:file "_package_IKTrackingService" :depends-on ("_package"))
    (:file "RobotStateQuery" :depends-on ("_package_RobotStateQuery"))
    (:file "_package_RobotStateQuery" :depends-on ("_package"))
    (:file "askReadiness" :depends-on ("_package_askReadiness"))
    (:file "_package_askReadiness" :depends-on ("_package"))
  ))