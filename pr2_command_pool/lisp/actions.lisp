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

(defparameter *general-action-client* nil)

(defun setup-move-robot-client ()
  (setf *general-action-client* (actionlib:make-action-client "/graspkard/move_robot" "suturo_manipulation_msgs/MoveRobotAction")))

(defun get-move-robot-goal-conv(joint-config controller-config keys-values)
  (get-move-robot-goal
   (get-joints joint-config)
   (get-controller-specs controller-config)
   keys-values))

(defun get-move-robot-goal(joints controller-specs keys-values)
  (let ((params (strings->keyvalues keys-values)))
    (actionlib:make-action-goal *general-action-client*
      :controlled_joints (make-array (length joints) :initial-contents joints)
      :controller_yaml controller-specs
      :params (make-array (length params) :initial-contents params))))

; TODO(cpo): Obviously this needs more work. :)
(defun move-robot-feedback-cb(msg)
  (pprint msg))

(defun action-move-robot (config-name controller-name &rest keys-values)
  (actionlib:call-goal
   *general-action-client*
   (get-move-robot-goal-conv config-name controller-name keys-values)
   :feedback-cb 'move-robot-feedback-cb))
