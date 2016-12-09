(in-package :pr2-command-pool-package)

(defun action-open-gripper (arm)
  "arm: 0 = left, 1 = right, 2 = both, open given gripper")

(defun action-close-gripper (arm strength)
  "arm: 0 = left, 1 = right, 2 = both, close given gripper with given strength")

(defun action-grasp-object (arm object-pose)
  "send command to giskard with given object pose, to move given arm to grasping position")

(defun action-lift-object (arm height)
  "send command to giskard to lift given arm(s) arm: 0 = left, 1 = right, 2 = both")

(defun action-put-down-object (arm location)
  "move given arm to location")

(defun action-get-in-base-pose ()
  "brings the pr2 into base pose")
