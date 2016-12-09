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

(defparameter *move-robot-action-client* nil)

(defun setup-move-robot-client ()
  (setf *move-robot-action-client* (actionlib:make-action-client "/graspkard/move_robot" "suturo_manipulation_msgs/MoveRobotAction")))

(defun get-move-robot-goal-conv(joint-config controller-config keys-values)
  (get-move-robot-goal
   (get-joint-config joint-config)
   (get-controller-specs controller-config)
   keys-values))

(defun get-move-robot-goal(joints controller-specs keys-values)
  (let ((params (strings->keyvalues keys-values)))
    (actionlib:make-action-goal *move-robot-action-client*
      :controlled_joints (make-array (length joints) :initial-contents joints)
      :controller_yaml controller-specs
      :params (make-array (length params) :initial-contents params))))

(defun move-robot-feedback-cb(msg)
  (with-fields
      (current_value alteration_rate)
      msg
    (format t "Error Value: ~a~%Alteration Rate: ~a~%~%" current_value alteration_rate)))

(defun handle-feedback-signal (signal)
  (let ((feedback-msg (actionlib:feedback signal)))
    (with-fields
        (current_value alteration_rate)
        feedback-msg
      (when (or (< current_value 0.05) (< alteration_rate 0.001))
        (invoke-restart 'actionlib:abort-goal)))))

(defun action-move-robot (config-name controller-name &rest keys-values)
  (handler-bind ((actionlib:feedback-signal #'handle-feedback-signal))
    (actionlib:send-goal-and-wait
     *move-robot-action-client*
     (get-move-robot-goal-conv config-name controller-name keys-values)
     :feedback-cb 'move-robot-feedback-cb)))
