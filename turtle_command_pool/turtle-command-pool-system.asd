(defsystem turtle-command-pool-system
  :depends-on (roslisp cl-tf std_msgs-msg actionlib_msgs-msg move_base_msgs-msg actionlib)
  :components
  ((:module "lisp"
    :components
    ((:file "package")
     (:file "commands" :depends-on ("package"))
     ))))
