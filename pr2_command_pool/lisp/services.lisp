(in-package :pr2-command-pool-package)

(defun service-is-object-in-view (object-id))

(defun service-get-object-location (object-id))

(defun action-grasp-object (location arm))

(defun action-lift-object (arm))

(defun service-get-drop-location (drop-zone)
  "Maybe this is redundant to service-get-object-location.")

(defun action-put-object-down-to (location arm)
  "Maybe this is redundant to action-grasp-object.")

(defun action-open-gripper (arm))

(defun action-get-into-home-pose ())


