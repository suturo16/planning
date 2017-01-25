(in-package :plan-execution-package)

(def-process-module giskard-manipulation (action-designator)
  (roslisp:ros-info (pr2-process-modules)
                    "Giskard manipulation invoked with action designator `~a'." action-designator)
  (destructuring-bind (command specs) (reference action-designator)
    (ecase command
      (base-pose
       (pr2-do::get-in-base-pose))
      (grasp
       (let ((arm (car (cdr (assoc 'arm specs))))
             (obj-info (car (cdr (assoc 'obj-info specs)))))
         (grasp-object obj-info arm)))
      (place
       (let ((arm (cdr (assoc 'arm specs)))
             (obj-info (cdr (assoc 'obj-info specs)))
             (target-info (cdr (assoc 'target specs))))
         (place-object obj-info target-info arm)))
      (cut
       nil)
      (test
       nil))))

(defmacro with-pr2-process-modules (&body body)
  `(with-process-modules-running
       (giskard-manipulation)
     ,@body))
