(in-package :plan-execution-package)

(defun execute (task)
  "Start a node and execute the given task."
  (common:ensure-node-is-running)
  (execute-task task))
  

(cram-language:def-top-level-cram-function execute-task (task)
  "Execute the given task in semrec context."
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
  "Execute the given designators with the manipulation pm."
  (when desigs
    (pm-execute :manipulation (car desigs))
    
    ; Call the function recursively with the rest of the list.
    (execute-desigs (cdr desigs))))

(defun task->designators (task)
  "Use a simple switch to translate a task to a list of designators."
  (alexandria:switch (task :test #'equal)
    ("grasp cylinder"
     (list (make-designator :action `((:type :grasp) (:arm ,pr2-do::+right-arm+) (:object "Cylinder")))))
    ("grasp knife"
     (list (make-designator :action `((:type :grasp) (:arm ,pr2-do::+right-arm+) (:object "Knife")))))
    ("just cut"
      (make-designator :action `((:type :cut) (:arm ,pr2-do::+right-arm+) (:knife "Knife") (:cake "Box"))))
    ("cut cake"
     (list (make-designator :action `((:type :grasp) (:arm ,pr2-do::+right-arm+) (:object "Knife")))
           (make-designator :action `((:type :detach) (:arm ,pr2-do::+right-arm+) (:object "Knife")))
           (make-designator :action `((:type :cut) (:arm ,pr2-do::+right-arm+) (:knife "Knife") (:cake "Box")))))))


