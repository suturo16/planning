(in-package :plan-generator-package)


(defun generate-pddl-problem (name domain objects init-predicates goal-predicates)
"Generate a pddl-problem for the task of serving N pieces of cake."
	(create-pddl-file(concatenate 'string 
		(generate-header name domain) 			
		(generate-objects-section objects)
		(generate-init-section init-predicates) 
		(generate-goal-section goal-predicates))))


(defun generate-header (name domain)
"Generate the header for the problem NAME in domain DOMAIN."
  (format nil "(define (problem ~A) ~%  (:domain ~A)" name domain))


(defun generate-objects-section (objects)
"Generate the objects-section with OBJECTS."
	(format nil "~%~%  (:objects ~{~%    ~A~})" objects))


(defun generate-init-section (predicates)
"Generate the init-section with PREDICATES."
	(format nil "~%~%  (:init ~{~%~^    (~A~^)~}))" predicates))


(defun generate-goal-section (predicates)
"Generate the goal-section with PREDICATES."
	(format nil "~%~%  (:goal~%    (and ~{~%~^        (~A~^)~}))))" predicates))


(defun create-pddl-file (context)
"Create pddl-file containing CONTEXT."
	(with-open-file (str "/home/jasmin/documents/planning/plan generator/test/task.pddl"
                     	:direction :output
                    	:if-exists :supersede
                    	:if-does-not-exist :create)
  	(format str context)))


