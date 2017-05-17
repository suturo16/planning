(in-package :plan-execution-package)

(def-process-module giskard-manipulation (action-designator)
  "Perform an giskard manipulation action on base of ACTION-DESIGNATOR.

ACTION-DESIGNATOR (designator): description of the desired action in the form as an action designator."
  (roslisp:ros-info (pr2-process-modules)
                    "Giskard manipulation invoked with action designator `~a'." action-designator)
  ; Destruct the referenced designator into the specific command and the alist of specifications.
  (destructuring-bind (command specs) (reference action-designator)
    (ecase command
      (base-pose
       ; Just get in the base pose.
       (pr2-do::get-in-base-pose))

      (grasp
       ; Grasp the specified object with the specified arm.
       (let ((arm (car (cdr (assoc 'arm specs))))
             (obj-info (car (cdr (assoc 'obj-info specs)))))
         (grasp obj-info arm)))

      (place
       ; Place the specified object, which is in the gripper of the specified arm, on the specified target.
       (let ((arm (car (cdr (assoc 'arm specs))))
             (obj-info (car (cdr (assoc 'obj-info specs))))
             (target-info (car (cdr (assoc 'target specs)))))
         (place-object obj-info target-info arm)))

      (detach
       ; Detach object from rack.
       (let ((arm (car (cdr (assoc 'arm specs))))
             (obj-info (car (cdr (assoc 'obj-info specs)))))
         (detach-object-from-rack obj-info arm)))
      
      (cut
       ; CUT! CUT! CUT! CAKE CRUMBS EVERYWHERE!!!!
       (let ((arm (car (cdr (assoc 'arm specs))))
             (knife-info (car (cdr (assoc 'knife specs))))
             (cake-info (car (cdr (assoc 'cake specs)))))                        
         (cut-object arm knife-info cake-info)))

      (test
       ; Just a test stub to appease one's sanity.
       nil))))

(defmacro with-pr2-process-modules (&body body)
  "Wrapper for with-process-modules-running for the PR2 process modules."
  `(with-process-modules-running
       (giskard-manipulation)
     ,@body))
