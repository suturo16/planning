(define (problem cake-on-turtlebot) 
  (:domain caterros)

  (:objects 
    knife0 - knife
    spatula0 - spatula
    plate0 - plate
    cake0 - cake
    pieceofcake0 - pieceofcake)

  (:init 
    (free left)
    (free right)
    (at-rack knife0)
    (on-table cake0)
    (on-table plate0)
    (not-existing pieceofcake0))

  (:goal
    (and 
        (on-plate pieceofcake0 plate1)
        (on-turtlebot plate0))))