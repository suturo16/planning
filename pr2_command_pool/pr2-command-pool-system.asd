(defsystem pr2-command-pool-system
  :depends-on (roslisp std_msgs-msg actionlib actionlib_msgs-msg suturo_manipulation_msgs-msg cram-language cram-utilities cram-json-prolog cram-prolog)
  :components
  ((:module "lisp"
    :components
    ((:file "package")
     (:file "commands" :depends-on ("package"))
     (:file "services" :depends-on ("package"))
     (:file "topics" :depends-on ("package"))
     (:file "actions" :depends-on ("package"))
     (:file "utils" :depends-on ("package"))
     (:file "prolog" :depends-on ("package"))))))
