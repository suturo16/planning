(in-package :pr2-command-pool-package)

(defun close-gripper (arm &optional  (strength 1.0))
  "Let Raphael close the gripper with given id and maybe strength"
  (print arm)
  (print strength))

(defun open-gripper (arm)
  (print arm))

(defun is-object-in-view (object-id)
  T)

(defun get-object-location (object-id)
  (print object-id))

(defun move-arm-to-object (object-location arm)
  (print arm)
  (print object-location))

(defun get-drop-location (side)
  (print side))

(defun move-object-with-arm (location arm)
  (print arm)
  (print location))

(defun get-in-base-pose ()
  "brings the pr2 into base pose")
