(in-package :plan-generator-package)

"Objects available in the problem scenario."
(defvar objects '("knife - knife" "spatula - spatula" "plate - plate" "cake - cake"))

"Predicates describing the init state of the problem scenario."
(defvar init-predicates '("free left" "free right" "at-rack knife" "on-table cake" "on-table plate" "on-table spatula" "empty spatula"))

"Predicates describing the goal state of the problem scenario."
(defvar goal-predicates '("on-turtlebot plate"))

"Name of the domain to that the problem scenario belongs to."
(defvar domain "caterros")

"Name of the problem."
(defvar name "cake-on-turtlebot")

"Path to the folder where the generated file should be saved."
(defvar path "/home/jasmin/suturo_ws/src/planning/plan_generator/pddl/task.pddl")


(defun generate-plan-for-cake-serving (&optional (n 1))
"Generate a plan for serving N pieces of cake."
	(service-generate-plan "domain.pddl" (create-pddl-task-for-cake-serving n)))



(defun create-pddl-task-for-cake-serving (&optional (n 1))
"Create pddl file for the serving of pieces of cake.
Optionally specify amount N, default is 1."
	(generate-pddl-problem 
		name 
		domain 
		(add-pieces-of-cake n objects)
		(add-init-predicates-for-pieces-of-cake n init-predicates)
		(add-goal-predicates-for-pieces-of-cake n goal-predicates)
))


(defun add-pieces-of-cake (n objects)
"Add N pieces-of-cake to a list of OBJECTS."
 	(append 
		objects 
		(loop for i to (- n 1) 
			collect (concatenate 'string "pieceofcake" (write-to-string i) " - pieceofcake"))))


(defun add-init-predicates-for-pieces-of-cake (n predicates)
"Add init-predicates for N pieces-of-cake to a list of PREDICATES."
	(append 
		predicates 
		(loop for i to (- n 1) 
			collect (concatenate 'string "not-existing pieceofcake" (write-to-string i)))))


(defun add-goal-predicates-for-pieces-of-cake (n predicates)
"Add goal-predicates for N pieces-of-cake to a list of PREDICATES."
	(append 
		(loop for i to (- n 1) 
			collect (concatenate 'string "on-plate pieceofcake" (write-to-string i) " plate"))
		predicates))

