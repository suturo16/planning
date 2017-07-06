(defsystem planning-communication-system
  :depends-on (roslisp
               std_msgs-msg
               planning-common-system
               s-xml-rpc
               ip-interfaces)
  :components
  ((:module "lisp"
    :components
    ((:file "package")
     (:file "rpc-server" :depends-on ("package"))
     (:file "rpc-client" :depends-on ("package"))
     (:file "command-handler" :depends-on ("package"))
     (:file "init" :depends-on ("package" "rpc-server" "rpc-client" "command-handler"))))))
