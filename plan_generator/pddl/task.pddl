(define (problem cake-on-turtlebot) 
  (:domain caterros)

  (:objects 
    knife - knife
    spatula - spatula
    plate - plate
    cake - cake
    pieceofcake0 - pieceofcake)

  (:init 
    (free left)
    (free right)
    (at-rack knife)
    (on-table cake)
    (on-table plate)
    (on-table spatula) 
    (not-existing pieceofcake0))

  (:goal
    (and 
        (on-plate pieceofcake0 plate)
        (on-turtlebot plate))))
