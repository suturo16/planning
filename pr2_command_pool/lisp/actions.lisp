(in-package :pr2-command-pool-package)

(defparameter *move-robot-action-client* nil)

(defun setup-move-robot-client ()
  (setf *move-robot-action-client* (actionlib:make-action-client "/graspkard/move_robot" "suturo_manipulation_msgs/MoveRobotAction")))

(defun get-move-robot-goal-conv(joint-config controller-config typed-params)
  (get-move-robot-goal
   (get-joint-config joint-config)
   (get-controller-specs controller-config)
   typed-params))

(defun get-move-robot-goal(joints controller-specs typed-params)
  (actionlib:make-action-goal *move-robot-action-client*
    :controlled_joints (make-array (length joints) :initial-contents joints)
    :controller_yaml controller-specs
    :params (make-array (length typed-params) :initial-contents typed-params)))

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

(defun action-move-robot (client config-name controller-name &rest typed-params)
  (handler-bind ((actionlib:feedback-signal #'handle-feedback-signal))
    (actionlib:send-goal-and-wait
     client
     (get-move-robot-goal-conv config-name controller-name typed-params)
     :feedback-cb 'move-robot-feedback-cb)))

(defun action-move-gripper (target-width arm strength)
  (when (not (member arm (list +left-arm+ +right-arm+)))
    (ros-error "action-move-gripper" "Unsupported arm specification: ~a." arm))
  (let ((controller-name (format nil "pr2_~a_gripper" arm))
        (param-name (format nil "~a_gripper_effort" arm)))
    (action-move-robot *move-robot-action-client* "pr2_upper_body" controller-name
                       (make-param +double+ T "target-width" (write-to-string target-width))
                       (make-param +double+ T param-name (write-to-string strength)))))
