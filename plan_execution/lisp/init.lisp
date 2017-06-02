(in-package :plan-execution-package)

(defun init-planning (&key (my-ip NIL) (perception NIL))
  "Initialize everything planning needs to run.

MY-IP (string): Look at `pcomm::setup-pepper-communication's docstring for further information."
  (start-ros-node "planning")

  ;; Initialize communication with pepper
  (when my-ip
    (pcomm::setup-pepper-communication #'pexecution::execute my-ip))
  
  ;; Initialize action client
  (common::setup-move-robot-client)
  
  ;; Let Perception see.
  ;;(pr2-do::run-full-pipeline)
  (when perception
    (common:run-full-pipeline))
  (print "Done initializing. You can start planning now!"))
