(defsystem pr2-command-pool-system
  :depends-on (roslisp
               std_msgs-msg
               actionlib
               actionlib_msgs-msg
               suturo_manipulation_msgs-msg
               suturo_perception_msgs-srv
               suturo_knowledge_msgs-srv
               alexandria
               cl-tf
               cram-language
               cram-utilities
               cram-json-prolog
               planning-common-system)
  :components
  ((:module "lisp"
    :components
    ((:file "package")
     (:file "break-functions" :depends-on ("package"))
     (:file "actions" :depends-on ("package" "break-functions"))
     (:file "commands" :depends-on ("package" "actions" "break-functions"))))))
