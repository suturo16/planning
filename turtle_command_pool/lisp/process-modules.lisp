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

(defmacro with-turtle-process-modules (&body body)
  `(with-process-modules-running
       (turtle-navigation turtle-test-module)
     ,@body))

(defun test-fun ()
  (top-level
    (with-turtle-process-modules
      (process-module-alias :navigation 'turtle-test-module)
      (with-designators
          ((test-desig :action `((:type :move-to) (:location (1 2 3)) (:next-to "table1"))))
           (pm-execute :navigation test-desig)))))
