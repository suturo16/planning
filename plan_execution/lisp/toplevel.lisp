(in-package :plan-execution-package)

(defun execute (task)
  "Execute TASK. Start a node if necessary.

TASK (string): Natural language description of the task.
               As of now it has to be one of the strings defined in `task->designators'."
  (common:ensure-node-is-running)
  (format T "Execute command: ~a" task)
  (execute-task task))
  

(cram-language:def-top-level-cram-function execute-task (task)
  "Execute TASK in semrec context."
  (roslisp::ensure-node-is-running)
  
  ;; settings for semrec
  (beliefstate:enable-logging T)
  (beliefstate::register-owl-namespace "knowrob" 
                                       "http://knowrob.org/kb/knowrob.owl#"  cpl-impl::log-id)
  (beliefstate::register-owl-namespace "cram_log" 
                                       "http://knowrob.org/kb/cram_log.owl#" cpl-impl::log-id)

  ;; Use the PR2 process modules.
  (with-pr2-process-modules
    ;; Give our pm an alias, so it's less of a hassle to call it later.
    (process-module-alias :manipulation 'giskard-manipulation)

    (turtle-do::with-turtle-process-modules
    (process-module-alias :navigation 'turtle-do::simple-navigation))
    
    
    ;; Translate the task to designators and execute
    (format t "DESIG-PROP ~a~%" (cram-prolog:prolog (list 'desig-prop (car (task->designators "turtle deliver")) '(:type ?x))))
    (format t "MATCHING-PROCESS-MODULE ~a~%" (cram-prolog:prolog (list 'matching-process-module (car (task->designators "turtle deliver")) '?y)))
    (execute-desigs (task->designators task)))
  
  



  
  ;; logs extraction
  (beliefstate::set-experiment-meta-data
   "performedInMap" 
   "http://knowrob.org/kb/IAI-kitchen.owl#IAIKitchenMap_PM580j" :type 
   :resource :ignore-namespace t)
  
  ;;(beliefstate:extract-files)
  )

(defun execute-desigs (desigs)
  "Execute DESIGS with the manipulation pm.

DESIG (list of designators): List of designators to be executed."
  (if desigs
      (seq
        (pm-execute-matching (car desigs))
        
        ;; Call the function recursively with the rest of the list.
        (execute-desigs (cdr desigs)))
      ;; else show victory message
      (print "Done executing!")))

(defun task->designators (task)
  "Translate TASK to a list of designators."
  (alexandria:switch (task :test (lambda (x xs) (member x xs :test #'equal)))
    ('("basepose" "step0")
     (list (make-designator :action `((:type :base-pose)))))
    ('("grasp cylinder")
     (list (make-designator :action `((:type :grasp) (:arm ,common:+right-arm+) (:object "cylinder")))))
    ('("grasp knife" "step1")
     (list (make-designator :action `((:type :grasp) (:arm ,common:+right-arm+) (:object "cakeKnife")))))
    ('("detach" "step2")
     (list (make-designator :action `((:type :detach) (:arm ,common:+right-arm+) (:object "cakeKnife")))))
    ('("move spatula next to cake" "step3")
     (list (make-designator :action `((:type :move-with-arm) (:arm ,common:+left-arm+) (:object "SupportingPlaneOfCakeSpatula1") (:target "next2cake")))))
    ('("grasp spatula")
      (list (make-designator :action `((:type :grasp) (:arm ,common:+left-arm+) (:object "cakeSpatula")))))
    ('("just cut" "step4b")
     (list (make-designator :action `((:type :cut) (:arm ,common:+right-arm+) (:knife "cakeKnife") (:cake "box")))))
    ('("just cut and move" "step4")
     (list (make-designator :action `((:type :cut) (:arm ,common:+right-arm+) (:knife "cakeKnife") (:cake "box") (:target "cakeSpatula")))))
    ('("move n flip" "step5")
     (list (make-designator :action `((:type :move-n-flip) (:arm ,common:+left-arm+) (:tool "cakeSpatula") (:target "dinnerPlateForCake")))))
    ('("drop spatula" "step6")
     (list (make-designator :action `((:type :move-with-arm) (:arm ,common:+left-arm+) (:object "cakeSpatula") (:target "spatulaDropZone")))
           (make-designator :action `((:type :move-gripper) (:arm ,common:+left-arm+) (:target "open")))
           (make-designator :action `((:type :release) (:arm ,common:+left-arm+)))))
    ('("grasp plate" "step7")
     (list (make-designator :action `((:type :grasp) (:arm ,common:+left-arm+) (:object "dinnerPlateForCake")))))
    ('("deliver plate" "step8")
     (list (make-designator :action `((:type :move-with-arm) (:arm ,common:+left-arm+) (:object "dinnerPlateForCake") (:target "deliver")))))
    ('("cut cake")
     (list (make-designator :action `((:type :grasp) (:arm ,common:+right-arm+) (:object "cakeKnife")))
           (make-designator :action `((:type :detach) (:arm ,common:+right-arm+) (:object "cakeKnife")))
           (make-designator :action `((:type :cut) (:arm ,common:+right-arm+) (:knife "cakeKnife") (:cake "box")))))
    ('("prep")
     (list (make-designator :action `((:type :base-pose)))
           (make-designator :action `((:type :grasp) (:arm ,common:+right-arm+) (:object "cakeKnife")))
           (make-designator :action `((:type :detach) (:arm ,common:+right-arm+) (:object "cakeKnife")))
           (make-designator :action `((:type :grasp) (:arm ,common:+left-arm+) (:object "cakeSpatula")))))
    ('("cut")
      (list (make-designator :action `((:type :move-with-arm) (:arm ,common:+left-arm+) (:object "cakeSpatula") (:target "next2cake")))
            (make-designator :action `((:type :cut) (:arm ,common:+right-arm+) (:knife "cakeKnife") (:cake "box") (:target "SupportingPlaneOfCakeSpatula1")))
            (make-designator :action `((:type :move-n-flip) (:arm ,common:+left-arm+) (:tool "cakeSpatula") (:target "dinnerPlateForCake")))))
    ('("deliver")
      (list (make-designator :action `((:type :move-with-arm) (:arm ,common:+left-arm+) (:object "cakeSpatula") (:target "spatulaDropZone")))
            (make-designator :action `((:type :move-gripper) (:arm ,common:+left-arm+) (:target "open")))
            (make-designator :action `((:type :release) (:arm ,common:+left-arm+)))
            (make-designator :action `((:type :grasp) (:arm ,common:+left-arm+) (:object "dinnerPlateForCake")))
            (make-designator :action `((:type :move-with-arm) (:arm ,common:+left-arm+) (:object "dinnerPlateForCake") (:target "deliver")))))
    ('("turtle deliver")
      (list (make-designator :action `((:type :goal) (:go-to :table) (:goal
                                                                       ,(make-designator :location `((:go-to ,(cl-transforms-stamped:make-pose-stamped "map" (roslisp:ros-time)
                                                                          (cl-transforms:make-3d-vector -3.3 -5.5 0.0) (cl-transforms:make-quaternion 0.0 0.0 0.0 1.0))))))))))
    ('("demo")
     (list (make-designator :action `((:type :base-pose)))
           (make-designator :action `((:type :grasp) (:arm ,common:+right-arm+) (:object "cakeKnife")))
           (make-designator :action `((:type :detach) (:arm ,common:+right-arm+) (:object "cakeKnife")))
           (make-designator :action `((:type :grasp) (:arm ,common:+left-arm+) (:object "cakeSpatula")))
           (make-designator :action `((:type :move-with-arm) (:arm ,common:+left-arm+) (:object "cakeSpatula") (:target "next2cake")))
           (make-designator :action `((:type :cut) (:arm ,common:+right-arm+) (:knife "cakeKnife") (:cake "box") (:target "cakeSpatula")))
           (make-designator :action `((:type :move-n-flip) (:arm ,common:+left-arm+) (:tool "cakeSpatula") (:target "dinnerPlateForCake")))
           (make-designator :action `((:type :move-with-arm) (:arm ,common:+left-arm+) (:object "cakeSpatula") (:target "spatulaDropZone")))
           (make-designator :action `((:type :move-gripper) (:arm ,common:+left-arm+) (:target "open")))
           (make-designator :action `((:type :release) (:arm ,common:+left-arm+)))
           (make-designator :action `((:type :grasp) (:arm ,common:+left-arm+) (:object "dinnerPlateForCake")))
           (make-designator :action `((:type :move-with-arm) (:arm ,common:+left-arm+) (:object "dinnerPlateForCake") (:target "deliver")))))
    ('("move cake hack")
      (list (make-designator :action `((:type :move-with-arm) (:arm ,common:+right-arm+) (:object "cakeKnife") (:target "SupportingPlaneOfSpatula1")))))))


(defun task-reference (task)
  (reference-desigs (task->designators task)))

(defun reference-desigs (desigs)
  (if desigs
      (progn
        (print (reference (car desigs)))
        (reference-desigs (cdr desigs)))
      (print "Done referencing!")))
