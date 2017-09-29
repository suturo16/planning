;;
;; PDDL file describing the domain of the caterros cafe
;;

(define (domain caterros)
(:requirements :typing)
(:types knife spatula plate - tool
        tool cake pieceofcake - object
        gripper)
(:constants left right - gripper)
            
(:predicates (at-rack ?t - tool)
             (on-table ?o - object)
             (at-gripper ?t - tool ?g - gripper) 
	     (on-spatula ?poc - pieceofcake ?s - spatula)
             (on-plate ?poc - pieceofcake ?p - plate) 
             (on-turtlebot ?t - tool)
	     (next-to-cake ?t - tool ?c - cake)
             (free ?g - gripper) ;; gripper g does not hold any tool
	     (empty ?s - spatula) ;; nothing is laying on the spatula
             (not-existing ?poc - pieceofcake))

;; Use gripper g to grasp tool t.
;; Tools can be grasped from the rack or the table.
(:action grasp-tool 
    :parameters (?g - gripper ?t - tool) 
    :precondition   (and  
                        (free ?g)
                        (not (on-turtlebot ?t)))
    :effect         (and 
                        (at-gripper ?t ?g) 
                        (not (on-table ?t))
                        (not (free ?g))))

;; Use gripper g to detach tool t from the rack.
(:action detach-tool-from-rack
    :parameters (?g - gripper ?t - tool) 
    :precondition   (and  
                        (at-gripper ?t ?g) 
                        (at-rack ?t)) 
    :effect         (and 
                        (not (at-rack ?t))))

;; Hold tool t at gripper g next to cake c.
(:action hold-next-to-cake
    :parameters (?g - gripper ?t - tool ?c - cake) 
    :precondition   (and  
                        (at-gripper ?t ?g) 
                        (not (at-rack ?t))) 
    :effect         (and 
                        (next-to-cake ?t ?c)))

;; Use knife k at gripper g1 to cut a pieceofcake poc from cake c.
;; Use spatula s at gripper g2 next to c to put poc on it.
(:action cut-cake 
    :parameters (?g1 - gripper ?k - knife ?c - cake ?s - spatula  ?g2 - gripper  ?poc - pieceofcake)
    :precondition   (and   
                        (on-table ?c)
                        (at-gripper ?k ?g1)
		  	(at-gripper ?s ?g2)
                        (not (at-rack ?k))
                        (next-to-cake ?s ?c)
			(empty ?s)
                        (not-existing ?poc))
    :effect         (and 
                        (not (not-existing ?poc))
			(not (empty ?s))
                        (on-spatula ?poc ?s)))

;; Lay spatula s at gripper g down on the table.
(:action place-spatula-on-table
   :parameters (?g - gripper ?s - spatula)
   :precondition    (and    
                        (at-gripper ?s ?g)
                        (not (at-rack ?s)))
   :effect          (and    
                        (on-table ?s) 
                        (not (at-gripper ?s ?g))
		        (free ?g)))

;; Use spatula s at gripper g to place the pieceofcake poc on the plate p.
;; Only works if poc is already laying on s.
(:action place-piece-of-cake-on-plate 
    :parameters (?g - gripper ?s - spatula ?p - plate ?poc - pieceofcake)
    :precondition   (and    
                        (on-table ?p)
                        (on-spatula ?poc ?s)
			(at-gripper ?s ?g))
    :effect         (and    
                        (on-plate ?poc ?p)
			(empty ?s) 
                        (not (on-spatula ?poc ?s))))

;; Place the plate p at gripper g on the turtlebot.                        
(:action place-plate-on-turtlebot 
    :parameters (?g - gripper ?p - plate)
    :precondition   (and     
                        (at-gripper ?p ?g))
    :effect         (and     
                        (on-turtlebot ?p) 
                        (not (at-gripper ?p ?g))
			(free ?g))))
