(defsystem plan-execution-system
  :depends-on (roslisp
               std_msgs-msg
               pr2-command-pool-system
               turtle-command-pool-system
               cram-language
               cram-designators
               cram-prolog
               cram-process-modules
               cram-language-designator-support
               )
  :components
  ((:module "lisp"
    :components
    ((:file "package")
     (:file "plans" :depends-on ("package"))
     (:file "action-designators" :depends-on ("package"))
     (:file "process-modules" :depends-on ("package" "plans"))
     (:file "toplevel" :depends-on ("package" "process-modules"))))))
