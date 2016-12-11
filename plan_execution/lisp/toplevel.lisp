(in-package :plan-execution-package)

;"in order to prevent the plan from starting again when it is already running
(defvar *plan-execution-running* NIL)
(defvar *plans-list*)

(defun reset-values-for-testing ()
  (setq *plan-execution-running* NIL)
  (setq *plans-list* NIL)
  (print "values reset"))
  

(defun execute (plan)
  (if *plan-execution-running*
      (progn
        (setq *plans-list* (list *plans-list* plan))
        (print "a plan is already being executed. Wait until it's done before sending a new command"))
      (progn
        (print "started executing toplevelplan")
        (setq *plan-execution-running* T)
;       (sleep 5) << for testing only. Might be still usefull
        (print plan)
        (if (string= (first plan) "grasp-object")
           (grasp-object "cylinder" pr2-do::+right-arm+)
           (ros-info (plan-execution-system) "First plan was not pick-up-object. Won't do anything then."))
       (if (string= (second plan) "place-object")
           (place-object (third plan) "cylinder" pr2-do::+right-arm+)
           (ros-info (plan-execution-system) "Second plan was not  put-down-object. Won't do anything then."))
       (pr2-do::get-in-base-pose)
       (print "toplevel plan finished")
       (setq *plan-execution-running* NIL))))
