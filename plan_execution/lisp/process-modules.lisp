(in-package :plan-execution-package)

(def-process-module giskard-manipulation (action-designator)
  (roslisp:ros-info (pr2-process-modules)
                    "Giskard manipulation invoked with action designator `~a'." action-designator)
  (destructuring-bind (command &optional obj-info arm target) (reference action-designator)
    (ecase command
      (base-pose
       (pr2-do::get-in-base-pose))
      (grasp
       (grasp-object (pr2-do::object-info-name obj-info) arm))
      (place
       nil)
      (cut
       nil)
      (test
       arm))))

(defmacro with-pr2-process-modules (&body body)
  `(with-process-modules-running
       (giskard-manipulation)
     ,@body))
