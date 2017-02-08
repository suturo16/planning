(in-package :plan-execution-package)

;"in order to prevent the plan from starting again when it is already running
;(let ((executing (make-fluent :name "executing" :value nil))))
(defvar *executing* (make-fluent :name "executing" :value nil))
(defvar *plans-list* nil)

;idea: have a q of plans
(defun execute (plan)
  (push plan *plans-list*)
  
  (print "wait-for-executing...")
    (wait-for *executing*)

  ;checks first if a fucntion with a matching name exists, then tries to call it.
   (progn
     (if (fboundp (intern (string-upcase (car (last *plans-list*))) 'plan-execution-package))
         (progn
           (funcall (intern (string-upcase (car (last *plans-list*))) 'plan-execution-package))))
           (print "This function is not defined, or the plan doesn't exist.")))


;fucntions for testing stuff
;this currently has to be the top lvl function, because one cannot nest these.
(def-top-level-cram-function testy ()
  (setf (value *executing*) NIL)
  (par
   (progn
     (execute "test-function"))
   (progn
     (sleep 5)
     (setf (value *executing*) T))))



;chris stuff
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
