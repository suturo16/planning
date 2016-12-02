(defsystem pepper-communication-system
  :depends-on (roslisp std_msgs-msg plan-execution-system)
  :components
  ((:module "lisp"
    :components
    ((:file "package")
     (:file "command-executer" :depends-on ("package"))
     (:file "command-listener" :depends-on ("package"))
     ))))
