(in-package :pr2-command-pool-package)

(defun get-into-base-pose ()
  "Moves the pr2 into base pose, holding his hands over his shoulders.")

(defun close-gripper (gripper-id &optional  (strength 1.0))
  "Let Raphael close the gripper with given id and maybe strength"
  (print gripper-id)
  (print strength))

(defun open-gripper (arm)
  "Opens given grippper (0 = left, 1 = right, 2 = both).")

(defun grasp-object (location arm)
  "Moves arm (0 = left, 1 = right) to given grasping position.")

(defun lift-object (arm)
  "Lifts given arm a bit (0 = left, 1 = right).")

(defun put-object-down-to (location arm)
  "Moves the given arm (0 = left, 1 = right) to the given location.")

(defun is-object-in-view (object-id)
  "Returns T, if the object with given id is currently in vision.")

(defun get-object-location (object-id)
  "Returns the position of the object with given id.")

(defun get-drop-location (drop-zone)
  "Returns area as koordinates, depending on the color of the drop-zone (green or red).")

