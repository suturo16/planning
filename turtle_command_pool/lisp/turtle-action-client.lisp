(in-package :turtle-do)

(defvar *move-base-client* nil)

(defun init-action-client ()
  (setf *move-base-client* (actionlib:make-action-client
                            "tortugabot1/move_base"
                            "move_base_msgs/MoveBaseAction"))
  (roslisp:ros-info (navigate-map)
                    "Waiting for move_base action server...")
  ;; workaround for race condition in actionlib wait-for server
  (loop until (actionlib:wait-for-server *move-base-client*))
  (roslisp:ros-info (navigate-map) 
                    "move_base action client created."))

(defun get-action-client ()
  (when (null *move-base-client*)
    (init-action-client))
  *move-base-client*)

(defun make-move-base-goal (pose-stamped-goal)
  (print "make-move-bse-goal")
  (actionlib:make-action-goal (get-action-client)
    target_pose pose-stamped-goal))

(defun call-move-base-action (frame-id translation rotation)
  (print "call-move-base-action")
  (unless (eq roslisp::*node-status* :running)
    (roslisp:start-ros-node "move-base-lisp-client"))

  (multiple-value-bind (result status)
      (let ((actionlib:*action-server-timeout* 10.0)
            (the-goal (cl-tf:to-msg
                       (cl-tf:make-pose-stamped
                        frame-id
                        (roslisp::ros-time)
                        translation rotation))))
        (actionlib:call-goal
         (get-action-client)
         (make-move-base-goal the-goal)))
    (roslisp:ros-info (navigate-map) "Move_base action finished.")
    (values result status)))

(def-cram-function execute-navigation-action (goal-pose)
  (print "execute-navigation-action")
    (unless (eq roslisp::*node-status* :running)
    (roslisp:start-ros-node "tortugabot1/move-base-lisp-client"))

  (multiple-value-bind (result status)
      (let ((actionlib:*action-server-timeout* 10.0))
        (actionlib:call-goal
         (get-action-client) goal-pose))
    (roslisp:ros-info (navigate-map) "Move_base action finished.")
    (values result status)))
