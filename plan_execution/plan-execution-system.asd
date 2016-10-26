(defsystem plan-execution-system
  :depends-on (roslisp
               std_msgs-msg
               pepper-command-pool-system
               pr2-command-pool-system
               turtle-command-pool-system)
  :components
  ((:module "lisp"
    :components
    ((:file "package")
     (:file "toplevel" :depends-on ("package"))
     ))))
