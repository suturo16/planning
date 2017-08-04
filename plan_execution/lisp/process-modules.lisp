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
       ;; Just get in the base pose.
       (pr2-do:get-in-base-pose))

      (move-gripper
       ;; Open or close the gripper.
       (let ((arm (car (cdr (assoc 'arm specs))))
             (target (car (cdr (assoc 'target specs)))))
         (if (string-equal "open" target)
             (pr2-do:open-gripper arm)
             (pr2-do:close-gripper arm))))

      (release
       ;; Open the gripper and move the arm away.
       (let ((arm (car (cdr (assoc 'arm specs)))))
         (pr2-do:release arm 0.09)))

      (grasp
       ;; Grasp the specified object with the specified arm.
       (let ((arm (car (cdr (assoc 'arm specs))))
             (obj-info (car (cdr (assoc 'obj-info specs)))))
         (grasp obj-info arm)))

      (move-with-arm
       ;; Move object in gripper of arm to a location.
       (let ((arm (car (cdr (assoc 'arm specs))))
             (obj-info (car (cdr (assoc 'object-info specs))))
             (target-info (car (cdr (assoc 'target-info specs)))))
         (place-object obj-info target-info arm)))

      (place
       ;; Place the specified object, which is in the gripper of the specified arm, on the specified target.
       (let ((arm (car (cdr (assoc 'arm specs))))
             (obj-info (car (cdr (assoc 'obj-info specs))))
             (target-info (car (cdr (assoc 'target specs)))))
         (place-object obj-info target-info arm T)))

      (detach
       ;; Detach object from rack.
       (let ((arm (car (cdr (assoc 'arm specs))))
             (obj-info (car (cdr (assoc 'obj-info specs)))))
         (detach-object-from-rack obj-info arm)))
      
      (cut
       ;; CUT! CUT! CUT! CAKE CRUMBS EVERYWHERE!!!!
       (let ((arm (car (cdr (assoc 'arm specs))))
             (knife-info (car (cdr (assoc 'knife specs))))
             (cake-info (car (cdr (assoc 'cake specs))))
             (target-info (car (cdr (assoc 'target specs)))))
         (cut-object arm knife-info cake-info target-info)))

      (move-n-flip
       ;; Move the tool above the target and flip it.
       (let ((arm (car (cdr (assoc 'arm specs))))
             (tool-info (car (cdr (assoc 'tool specs))))
             (target-info (car (cdr (assoc 'target specs)))))
         (move-n-flip arm tool-info target-info)))

      (test
       ; Just a test stub to appease one's sanity.
       nil)
      )))

(defmacro with-pr2-process-modules (&body body)
  "Wrapper for with-process-modules-running for the PR2 process modules."
  `(with-process-modules-running
       (pexecution:giskard-manipulation)
     ,@body))


(def-fact-group navigation-process-modules (available-process-module
                                            matching-process-module)
  (<- (available-process-module simple-navigation))
  (<- (available-process-module giskard-manipulation))
  
  (<- (matching-process-module ?desig simple-navigation)
    (desig-prop ?desig (:type :goal)))
  ;(<- (matching-process-module ?desig giskard-manipulation)
  ;(desig-prop ?desig (:type ?type)))
  )
;; alternatively define all the types here. it would be safer that way.
