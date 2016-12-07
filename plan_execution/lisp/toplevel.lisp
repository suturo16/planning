(in-package :plan-execution-package)

(defun execute (plan)
  "Start toplevel plan here."
  (print plan)
  (if (string= (first plan) "pick-up-object")
      (pick-up-object "cylinder" (third plan))
      (ros-info (plan-execution-system) "First plan was not pick-up-object. Won't do anything then."))
  (if (string= (second plan) "put-down-object")
      (put-down-object "right" (third plan))
      (ros-info (plan-execution-system) "Second plan was not  put-down-object. Won't do anything then.")))
