(in-package :plan-execution-package)

;"in order to prevent the plan from starting again when it is already running
(defvar *plan-execution-running* NIL)


(defun reset-values-for-testing ()
  (setq *plan-execution-running* NIL)
  (print "values reset"))


(define-condition toplvl-being-executed (simple-condition) ()
  (:report (lambda (condition stream)
             (princ "toplevel plan is currently being executed. Please try again later" stream))))
(define-condition toplvl-completed-execution (simple-condition) ()
  (:report (lambda (condition stream)
       (princ "toplevel finished executing a plan."))))


(defun execute (plan)
  (if *plan-execution-running*
      (progn
        (signal 'toplvl-being-executed))
      (progn
        (print "started executing toplevelplan")
        (setq *plan-execution-running* T)
;       (sleep 5) ;<< for testing only. Might be still usefull
        (print plan)
        (print (first plan))
        (if (string= (first plan) "grasp-object")
            (grasp-object "cylinder" pr2-do::+right-arm+)
            (ros-info (plan-execution-system) "First plan was not pick-up-object. Won't do anything then."))
        (if (string= (second plan) "place-object")
            (place-object (third plan) "cylinder" pr2-do::+right-arm+)
            (ros-info (plan-execution-system) "Second plan was not  put-down-object. Won't do anything then."))
        (pr2-do::get-in-base-pose)
        (print "toplevel plan finished")
        (setq *plan-execution-running* NIL)
        (signal 'toplvl-completed-execution))))

(cram-language:def-top-level-cram-function execute-cram (task)
  (when (eq (node-status) :SHUTDOWN)
    (start-ros-node "planning"))
  (with-pr2-process-modules
    (process-module-alias :manipulation 'giskard-manipulation)
    (execute-desigs (task->designators task))))

(defun execute-desigs (desigs)
  (when desigs
    (pm-execute :manipulation (car desigs))
    (execute-desigs (cdr desigs))))

(defun task->designators (task)
  (ecase task
    (:grasp-cylinder
     (list
      (make-designator :action `((:type :grasp) (:arm ,pr2-do::+right-arm+) (:object "cylinder")))))
    (:cut
     (list
      (make-designator :action `((:type :grasp) (:arm ,pr2-do::+right-arm+) (:object "knife")))
      (make-designator :action `((:type :cut) (:arm ,pr2-do::+right-arm+) (:knife "knife") (:cake "cake")))))))
