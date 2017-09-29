(defsystem plan-execution-system
  :depends-on (cram-language
               roslisp
               std_msgs-msg
               planning-communication-system
               pr2-command-pool-system
               turtle-command-pool-system
               planning-common-system
               plan-generator-system
               cram-designators
               cram-prolog
               cram-process-modules
               cram-language-designator-support
               cram-beliefstate)
               
  :components
  ((:module "lisp"
    :components
    ((:file "package")
     (:file "plans" :depends-on ("package"))
     (:file "action-designators" :depends-on ("package"))
     (:file "process-modules" :depends-on ("package" "plans"))
     (:file "toplevel" :depends-on ("package" "process-modules" "action-designators"))
     (:file "init" :depends-on ("package" "toplevel"))
     (:file "manager" :depends-on ("package" "toplevel" "init"))))))
