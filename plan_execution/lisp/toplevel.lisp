(in-package :plan-execution-package)

(defun execute (task)
  "Execute TASK. Start a node if necessary.

TASK (string): Natural language description of the task.
               As of now it has to be one of the strings defined in `task->designators'."
  (common:ensure-node-is-running)
  (format T "command: ~a" task)
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
    
    ;; Translate the task to designators and execute them.
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
        (pm-execute :manipulation (car desigs))
        
        ;; Call the function recursively with the rest of the list.
        (execute-desigs (cdr desigs)))
      ;; else show victory message
      (print "Done executing!")))

(defun task->designators (task)
  "Translate TASK to a list of designators."
  (alexandria:switch (task :test #'equal)
    ("grasp cylinder"
     (list (make-designator :action `((:type :grasp) (:arm ,common::+right-arm+) (:object "cylinder")))))
    ("grasp knife"
     (list (make-designator :action `((:type :grasp) (:arm ,common::+right-arm+) (:object "knife")))))
    ("move spatula next to cake"
     (list (make-designator :action `((:type :move-with-arm) (:arm ,common:+right-arm+) (:object "spatula") (:target "next2cake")))))
    ("just cut"
      (list (make-designator :action `((:type :cut) (:arm ,common::+right-arm+) (:knife "knife") (:cake "box")))))
    ("cut cake"
     (list (make-designator :action `((:type :grasp) (:arm ,common::+right-arm+) (:object "knife")))
           (make-designator :action `((:type :cut) (:arm ,common::+right-arm+) (:knife "knife") (:cake "box")))))
    ("demo"
     (list (make-designator :action `((:type :grasp) (:arm ,common:+right-arm+) (:object "knife")))
           (make-designator :action `((:type :grasp) (:arm ,common:+left-arm+) (:object "spatula")))
           (make-designator :action `((:type :move-with-arm) (:arm ,common:+left-arm+) (:object "spatula_shovel") (:target "next2cake")))
           (make-designator :action `((:type :cut) (:arm ,common:+right-arm+) (:knife "knife") (:cake "box") (:target "spatula_shovel")))
           (make-designator :action `((:type :move-n-flip) (:arm ,common:+left-arm+) (:tool "spatula_shovel") (:target "plate")))
           (make-designator :action `((:type :place) (:arm ,common:+left-arm+) (:object "spatula") (:target "next2cake")))
           (make-designator :action `((:type :grasp) (:arm ,common:+left-arm+) (:object "plate")))
           (make-designator :action `((:type :move-with-arm) (:arm ,common:+left-arm+) (:object "plate") (:target "deliver")))))))

