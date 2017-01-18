(in-package :pr2-command-pool-package)

(def-process-module giskard-manipulation (action-designator)
  (roslisp:ros-info (pr2-process-modules)
                    "Giskard manipulation invoked with action designator `~a'." action-designator)
  (destructuring-bind (command action-goal) (reference action-designator)))
