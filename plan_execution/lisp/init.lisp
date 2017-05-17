(in-package :plan-execution-package)

(defun init-planning (my-ip)
  "Initialize everything planning needs to run.

MY-IP (string): Look at `pcomm::setup-pepper-communication's docstring for further information."
  (start-ros-node "planning")

  ;; Initialize communication with pepper
  (pcomm::setup-pepper-communication #'execute my-ip)
  
  ;; Initialize action client
  (common::setup-move-robot-client)
  
  ;; Let Perception see.
  ;;(pr2-do::run-full-pipeline)
  (pr2-do::service-run-pipeline "Knife"))
