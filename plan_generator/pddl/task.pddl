(define (problem cake-on-turtlebot) 
  (:domain caterros)

  (:objects 
    knife - knife
    spatula - spatula
    plate - plate
    cake - cake
    pieceofcake0 - pieceofcake
    pieceofcake1 - pieceofcake)

  (:init 
    (free left)
    (free right)
    (at-rack knife)
    (on-table cake)
    (on-table plate)
    (on-table spatula)
    (empty spatula)
    (not-existing pieceofcake0)
    (not-existing pieceofcake1))

  (:goal
    (and 
        (on-plate pieceofcake0 plate)
        (on-plate pieceofcake1 plate)
        (on-turtlebot plate))))