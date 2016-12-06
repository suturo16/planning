(in-package :plan-execution-package)

(defun execute (plan)
  "Start toplevel plan here."
  (print plan)
  (if (string= (car plan) "pick-up-object")
      (pick-up-object "zylinder" "right")
      (ros-info (plan-execution-system) "First plan was not pick-up-object. Won't do anything then.")
      )
   (if (string= (cdr plan) "put-down-object")
      (put-down-object "right" "right")
      (ros-info (plan-execution-system) "Second plan was not  put-down -object. Won't do anything then.")
      )

  )
