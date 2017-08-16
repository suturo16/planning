(defsystem turtle-command-pool-system
  :depends-on (cram-language
               roslisp
               std_msgs-msg
               actionlib_msgs-msg
               move_base_msgs-msg
               actionlib
               geometry_msgs-msg
               cl-transforms
               cram-designators
               cram-prolog
               cram-process-modules
               cram-language-designator-support
               cram-location-costmap
               cram-occupancy-grid-costmap)
  :components
  ((:module "lisp"
    :components
    ((:file "package")
     (:file "commands" :depends-on ("package"))
     (:file "action-designators" :depends-on ("package"))
     (:file "location-designators" :depends-on ("package"))
     (:file "process-modules" :depends-on ("package" "turtle-action-client" "action-designators" "commands"))
     (:file "plans" :depends-on ("package"))
     (:file "turtle-action-client" :depends-on ("package"))))))
