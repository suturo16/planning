(in-package :plan-execution-package)

;"in order to prevent the plan from starting again when it is already running
(defvar *plan-execution-running* NIL)

(defun reset-values-for-testing ()
  (setq *plan-execution-running* NIL)
  (print "values reset"))
  

(defun execute (plan)
  (if *plan-execution-running*
      (print "a plan is already being executed. Wait until it's done before sending a new command")
      (progn
        (print "started executing toplevelplan")
        (setq *plan-execution-running* T)
;       (sleep 5) << for testing only. Might be still usefull
        (print plan)
        (if (string= (first plan) "pick-up-object")
           (pick-up-object "cylinder" (third plan))
           (ros-info (plan-execution-system) "First plan was not pick-up-object. Won't do anything then."))
       (if (string= (second plan) "put-down-object")
           (put-down-object "right" (third plan))
           (ros-info (plan-execution-system) "Second plan was not  put-down-object. Won't do anything then."))
       (print "toplevel plan finished")
       (setq *plan-execution-running* NIL))))
