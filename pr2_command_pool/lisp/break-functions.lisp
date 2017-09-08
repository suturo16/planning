(in-package :pr2-command-pool-package)

(defparameter +get-in-base-pose-error-limit+ 0.09)
(defparameter +action-move-gripper-error-limit+ 0.005)
(defparameter +move-arm-to-object-error-limit+ 0.01)
(defparameter +move-object-with-arm-error-limit+ 0.025)
(defparameter +move-n-flip-object-with-arm-error-limit+ 0.01)
(defparameter +grasp-knife-error-limit+ 0.025)
(defparameter +grasp-plate-error-limit+ 0.125)
(defparameter +grasp-spatula-error-limit+ 0.12)
(defparameter +release-error-limit+ 0.0000001)
(defparameter +detach-knife-from-rack-error-limit+ 0.000001)
(defparameter +take-cutting-position-error-limit+ 0.07)
(defparameter +cut-cake-error-limit+ 0.00001)
(defparameter +move-slice-aside-error-limit+ 0.05)
(defparameter +look-at-error-limit+ 0.015)

(defun error-break-function (limit error-val)
  (< error-val limit))
