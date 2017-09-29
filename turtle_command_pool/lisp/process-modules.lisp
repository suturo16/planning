(in-package :turtle-do)


;(cram-language:top-level
  (def-process-module turtle-navigation (input-designator)
    (let ((goal-pose (reference input-designator)))
      (ros-info (turtle-process-module) "Turtlebot navigation invoked with action designator.")
      (or (execute-navigation-action goal-pose)
      (fail 'navigation-failed))))
 ;)

(def-process-module turtle-test-module (input-designator)
  (ros-info (turtle-test-module)
            "turtle navigation test with designator `~a'."
            input-designator)
  (destructuring-bind (command goal-pose) (reference input-designator)
    (ecase command
      (move-action
       (or
        (execute-navigation-action goal-pose)
        (fail 'navigation-failed))))))

(def-process-module simple-navigation (action-designator)
  (roslisp:ros-info (turtle-process-modules)
                    "Turtle simple navigation invoked with action designator `~a'."
                    action-designator)
  (destructuring-bind (command action-goal) (reference action-designator)
                       (ecase command
                         (go-to
                         (format t "action-goal: ~a" action-designator)
                          (format t "action-designator: ~a" (reference action-designator))
                       ;   (print (reference action-goal))
                          (when (typep action-goal 'location-designator)
                            (let ((target-point (reference action-goal)))
                              (roslisp:ros-info (turtle-process-modules)
                                                "going to point ~a" target-point)
                              (execute-navigation-action (cl-tf:to-msg target-point))))))))

(defmacro with-turtle-process-modules (&body body)
`(with-process-modules-running
(turtle-do:simple-navigation)
,@body))

;; point is a cl-tf pose tho
(defun goto-location (point)
 (top-level
    (with-turtle-process-modules
      (process-module-alias :navigation 'simple-navigation)
      (with-designators
          ((area :location `((:go-to ,point)))
           (goal :action `((:type :goal) (:go-to :table) (:goal ,area))))
        (pm-execute :navigation goal)))))
