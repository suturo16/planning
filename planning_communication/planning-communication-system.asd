(defsystem planning-communication-system
  :depends-on (roslisp
               std_msgs-msg
               planning-common-system
               s-xml-rpc)
  :components
  ((:module "lisp"
    :components
    ((:file "package")
     (:file "rpc-server" :depends-on ("package"))
     (:file "rpc-client" :depends-on ("package"))
     (:file "command-handler" :depends-on ("package"))))))
