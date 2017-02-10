(defsystem pepper-communication-system
  :depends-on (roslisp
               std_msgs-msg
               plan-execution-system
               planning-common-system
               s-xml-rpc)
  :components
  ((:module "lisp"
    :components
    ((:file "package")
     (:file "command-executer" :depends-on ("package"))
     (:file "command-listener" :depends-on ("package"))
     (:file "rpc-client" :depends-on ("package"))))))
