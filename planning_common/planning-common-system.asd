(defsystem planning-common-system
  :depends-on (roslisp
               actionlib
               alexandria
               std_msgs-msg
               suturo_perception_msgs-srv
               suturo_knowledge_msgs-srv
               suturo_manipulation_msgs-msg
               cl-tf
               cram-utilities
               cram-json-prolog
               cram-language
               sound_play-msg)
  :components
  ((:module "lisp"
    :components
    ((:file "package")
     (:file "conditions" :depends-on ("package"))
     (:file "objects" :depends-on ("package"))
     (:file "topics" :depends-on ("package"))
     (:file "services" :depends-on ("package"))
     (:file "actions" :depends-on ("package"))
     (:file "prolog" :depends-on ("package"))
     (:file "utils" :depends-on ("package"))
     (:file "tf-utils" :depends-on ("package"))
     (:file "yaml-utils" :depends-on ("package"))))))
