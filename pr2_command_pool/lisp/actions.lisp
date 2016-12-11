(in-package :pr2-command-pool-package)

(defparameter *move-robot-action-client* nil)
(defparameter *place-object-client* nil)
(defparameter *gripper-client* nil)

(defun setup-move-robot-client ()
  (setf *move-robot-action-client* (actionlib:make-action-client "/graspkard/move_robot" "suturo_manipulation_msgs/MoveRobotAction"))
  (setf *place-object-client* (actionlib:make-action-client "/graspkard/place_object" "suturo_manipulation_msgs/MoveRobotAction"))
  (setf *gripper-client* (actionlib:make-action-client "/graspkard/gripper" "suturo_manipulation_msgs/MoveRobotAction")))

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

(defun action-move-robot (client config-name controller-name &rest keys-values)
  (handler-bind ((actionlib:feedback-signal #'handle-feedback-signal))
    (actionlib:send-goal-and-wait
     client
     (get-move-robot-goal-conv config-name controller-name keys-values)
     :feedback-cb 'move-robot-feedback-cb)))

(defun action-move-gripper (type arm strength)
  (when (not (member type (list "open" "close") :test #'string=))
    (ros-error "action-move-gripper" "Unsupported movement type: ~a." type))
  (when (not (member arm (list +left-arm+ +right-arm+)))
    (ros-error "action-move-gripper" "Unsupported arm specification: ~a." arm))
  (let ((controller-name (format nil "pr2_~a_~a_gripper" type arm))
        (param-name (format nil "~a_gripper_effort" arm)))
    (action-move-robot *gripper-client* "pr2_upper_body" controller-name param-name (write-to-string strength))))
