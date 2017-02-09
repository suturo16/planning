(in-package :plan-execution-package)

;"in order to prevent the plan from starting again when it is already running
;(let ((executing (make-fluent :name "executing" :value nil))))
(defvar *executing* (make-fluent :name "executing" :value nil))
(defvar *plans-list* nil)

;idea: have a q of plans
(cram-language:def-top-level-cram-function execute (plan)  
  (execute-cram plan))
    


;chris stuff
(cram-language:def-cram-function execute-cram (task)
  (when (eq (node-status) :SHUTDOWN)
    (start-ros-node "planning"))
  (with-pr2-process-modules
    (process-module-alias :manipulation 'giskard-manipulation)
    (execute-desigs (intern (string-upcase (task->designators task))))))

(defun execute-desigs (desigs)
  (when desigs
    (pm-execute :manipulation (car desigs))
    (execute-desigs (cdr desigs))))

(defun task->designators (task)
  (ecase task
    ('grasp-cylinder
     (print "grasp")
     (list
      (make-designator :action `((:type :grasp) (:arm ,pr2-do::+right-arm+) (:object "cylinder")))))
    ('cut
     (print "cut")
     (list
      (make-designator :action `((:type :grasp) (:arm ,pr2-do::+right-arm+) (:object "knife")))
      (make-designator :action `((:type :cut) (:arm ,pr2-do::+right-arm+) (:knife "knife") (:cake "cake")))))))

