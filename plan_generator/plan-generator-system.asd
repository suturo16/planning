(defsystem plan-generator-system
  :depends-on (roslisp std_msgs-msg)
  :components
  ((:module "lisp"
    :components
    ((:file "package")
     (:file "plan-generator" :depends-on ("package"))
     ))))
