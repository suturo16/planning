(defpackage :pr2-command-pool-package
  (:nicknames :pr2-do)
  (:use :cl :roslisp :common)

  (:export

   ;; commands
   :close-gripper
   :open-gripper
   :disconnect-obj-from-arm
   :connect-obj-with-gripper
   :move-arm-to-object
   :move-object-with-arm
   :move-n-flip-object-with-arm
   :get-in-base-pose
   :grasp-knife
   :grasp-plate
   :grasp-spatula
   :release
   :detach-knife-from-rack
   :take-cutting-position
   :cut-cake
   :move-slice-aside
   :look-at
   :get-arm-in-base-pose))
