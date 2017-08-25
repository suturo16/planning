(in-package :planning-common-package)

(defparameter *move-robot-action-client* nil)

(defun setup-move-robot-client ()
  "Setup client for robot movement."
  (setf *move-robot-action-client* (actionlib:make-action-client "/movement_server/movement_server" "suturo_manipulation_msgs/MoveRobotAction")))

(defun get-move-robot-client ()
  "Check if the client is already set and connected to server.
If not, set it up."
  (if *move-robot-action-client*
      (unless (actionlib:connected-to-server *move-robot-action-client*)
        (setup-move-robot-client))
      (setup-move-robot-client))
  *move-robot-action-client*)

(defun get-move-robot-goal-conv(controller-config typed-params)
  "Use joints from JOINT-CONFIG and controller of CONTROLLER-CONFIG for goal creation."
  (get-move-robot-goal
   (get-controller-specs controller-config)
   typed-params))

(defun get-move-robot-goal(controller-specs typed-params)
  "Create goal of movement using JOINTS and CONTROLLER-SPECS"
  (actionlib:make-action-goal (get-move-robot-client)
    :controller_yaml controller-specs
    :feedbackValue "feedback"
    :params (make-array (length typed-params) :initial-contents typed-params)))

(defun move-robot-feedback-cb(msg)
  "Print the current value and alteration rate contained in MSG."
  (with-fields
      (current_value alteration_rate)
      msg
    (format t "Error Value: ~a~%Alteration Rate: ~a~%~%" current_value alteration_rate)))

(defun action-move-robot
    (controller-name cb use-alt-rates &rest typed-params)
  "Call action with joints CONFIG-NAME and controller specification CONTROLLER-NAME.
Optionally takes function CB as a break condition for handling feedback signals and
an arbitrary number TYPED-PARAMS to use as params in the action call.

CONFIG-NAME (string): Name of the joint configuration to be used.
CONTROLLER-NAME (string): Name of the controller to be used.
CB (function): Break condition. See `make-feedback-signal-handler's documentation.
TYPED-PARAMS (suturo_manipulation_msgs-msg:TypedParam): Params to be send with the goal."
  (handler-bind ((actionlib:feedback-signal (make-feedback-signal-handler cb use-alt-rates)))
    (multiple-value-bind (result status)
        (actionlib:send-goal-and-wait
         (get-move-robot-client)
         (get-move-robot-goal-conv controller-name typed-params)
         :feedback-cb 'move-robot-feedback-cb
         :result-timeout 30
         :exec-timeout 30)
      (declare (ignore result))
      (alexandria:switch (status)
        (:ABORTED :SUCCESS)
        (:LOST (error 'common:action-lost))
        (:TIMEOUT (signal 'common:action-timeout))))))

(defparameter +alt-rate-limit+ 0.05)
(defparameter +min-alt-rate-count+ 5)

(defun make-feedback-signal-handler (&optional
                                       (error-cb (lambda (v) (< v 0.05)))
                                       (use-alt-rates NIL))
  "Return a function able to handle a action feedback signal from /movement_server/movement_server,
using CB as a break condition.

ERROR-CB (function): Break condition. (eg. '(lambda (v) (< v 0.05)))"
  (let (alt-rates)
    (lambda (signal)
      (let ((feedback-msg (actionlib:feedback signal)))
        (with-fields
            (current_value alteration_rate)
            feedback-msg
          (progn
            (setf alt-rates (cons alteration_rate alt-rates))
            (when (or
                   ;; when either the error value is low enough or the alteration rate stagnates abort the action
                   (funcall error-cb current_value)
                   (when (and use-alt-rates (>= (length alt-rates) +min-alt-rate-count+))
                     ;; if we should use the alteration rate and have enough samples, check it
                     (< (/ (reduce #'+ (subseq alt-rates 0 5)) +min-alt-rate-count+) +alt-rate-limit+)))
              (invoke-restart 'actionlib:abort-goal))))))))
