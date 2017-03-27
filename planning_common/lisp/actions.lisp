(in-package :planning-common-package)

(defparameter *move-robot-action-client* nil)

(defun setup-move-robot-client ()
  (setf *move-robot-action-client* (actionlib:make-action-client "/movement_server/movement_server" "suturo_manipulation_msgs/MoveRobotAction")))

(defun get-move-robot-client ()
  (if *move-robot-action-client*
      (unless (actionlib:connected-to-server *move-robot-action-client*)
        (setup-move-robot-client))
      (setup-move-robot-client))
  *move-robot-action-client*)

(defun get-move-robot-goal-conv(joint-config controller-config typed-params)
  (get-move-robot-goal
   (get-joint-config joint-config)
   (get-controller-specs controller-config)
   typed-params))

(defun get-move-robot-goal(joints controller-specs typed-params)
  (actionlib:make-action-goal (get-move-robot-client)
    :controlled_joints (make-array (length joints) :initial-contents joints)
    :controller_yaml controller-specs
    :feedbackValue "feedback"
    :params (make-array (length typed-params) :initial-contents typed-params)))

(defun move-robot-feedback-cb(msg)
  (with-fields
      (current_value alteration_rate)
      msg
    (format t "Error Value: ~a~%Alteration Rate: ~a~%~%" current_value alteration_rate)))

(defun action-move-robot
    (config-name controller-name &optional (cb (lambda (v) (< v 0.05))) &rest typed-params)
  (handler-bind ((actionlib:feedback-signal (make-feedback-signal-handler cb)))
    (actionlib:send-goal-and-wait
     (get-move-robot-client)
     (get-move-robot-goal-conv config-name controller-name typed-params)
     :feedback-cb 'move-robot-feedback-cb
     :result-timeout 12
     :exec-timeout 12)))

(defun make-feedback-signal-handler (cb)
  (lambda (signal)
    (let ((feedback-msg (actionlib:feedback signal)))
      (with-fields
          (current_value)
          feedback-msg
        (when (funcall cb current_value)
          (invoke-restart 'actionlib:abort-goal))))))
