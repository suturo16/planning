(in-package :pr2-command-pool-package)

(defparameter +break-functions+
  (alexandria:plist-alist
   (list
    "action-move-gripper" (lambda (v) (< v 0.017))
    "move-object-with-arm" (lambda (v) (< v 0.025))
    "move-n-flip-object-with-arm" (lambda (v) (< v 0.01))
    "get-in-base-pose" (lambda (v) (< v 0.09))
    "grasp-knife" (lambda (v) (< v 0.025))
    "grasp-plate" (lambda (v) (< v 0.125))
    "release" (lambda (v) (< v 0.0000001))
    "detach-knife-from-rack" (lambda (v) (< v 0.000001))
    "take-cutting-position" (lambda (v) (< v 0.07))
    "cut-cake" (lambda (v) (< v 0.025))
    "move-slice-aside" (lambda (v) (< v 0.015))
    "look-at" (lambda (v) (< v 0.015)))))
