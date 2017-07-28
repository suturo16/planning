(in-package :turtle-do)

(defstruct turtle-move
  location
  next-to)

(defparameter move-to-desig
  (desig:make-designator :action
                   '((:type :move-to)
                     (:location :location)
                     (:next-to :next-to))))

(cram-prolog:def-fact-group turtle-actions (action-desig)
  
  (<- (action-desig ?desig (move-action ((location ?location) (next-to ?next-to))))
    (desig-prop ?desig (:type :move-to))
    (desig-prop ?desig (:location ?location))
    (desig-prop ?desig (:next-to ?next-to))
    (lisp-fun make-turtle-move :location ?location :next-to ?next-to ?action))

  (<- (action-desig ?desig (test ((obj-info ?obj-info))))
    (desig-prop ?desig (:type :test)))
  )

(defun make-test-desig ()
  (defparameter test-desig (make-designator :action '((:type :move-to) (:location (1 2 3)) (:next-to "table1")))))
