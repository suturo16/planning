(in-package :plan-execution-package)

(defun init-planning (&key (use-pepper NIL) (perception NIL))
  "Initialize everything planning needs to run.

MY-IP (string): Look at `pcomm:setup-pepper-communication's docstring for further information."
  (start-ros-node "planning")
  ;(roslisp-utilities:startup-ros :name "lisp_node" :anonymous nil)
  
  ;; Initialize communication with pepper
  (when use-pepper
    (pcomm:setup-pepper-communication #'pexecution:execute))
  
  ;; Initialize action client
  (common:setup-move-robot-client)
  
  ;; Let Perception see.
  (when perception
    (common:run-full-pipeline))
  (print "Done initializing. You can start planning now!"))
