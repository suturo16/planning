(defsystem plan-generator-system
  :depends-on (roslisp std_msgs-msg)
  :components
  ((:module "lisp"
    :components
    ((:file "package")
     (:file "plan-generator" :depends-on ("package"))
     (:file "pddl-problem-generation" :depends-on ("package"))
     (:file "cake-serving-problem-generation" :depends-on ("package" "pddl-problem-generation"))
     ))))
