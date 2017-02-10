(defsystem planning-common-system
  :depends-on (roslisp
               std_msgs-msg
	       suturo_manipulation_msgs-msg
               alexandria
               cl-tf)
  :components
  ((:module "lisp"
    :components
    ((:file "package")
     (:file "utils" :depends-on ("package"))))))
