(in-package :plan-execution-package)

(def-fact-group move-robot-actions (action-desig)
  ; grasp
  (<- (action-desig ?desig (grasp ((arm ?arm) (obj-info ?obj-info))))
    (desig-prop ?desig (:type :grasp))
    (desig-prop ?desig (:arm ?arm))
    (desig-prop ?desig (:object ?object))
    (lisp-fun pr2-do::get-object-info ?object ?obj-info))

  ; place
  (<- (action-desig ?desig (place ((arm ?arm) (obj-info ?obj-info) (target ?target))))
    (desig-prop ?desig (:type :place))
    (desig-prop ?desig (:arm ?arm))
    (desig-prop ?desig (:target ?target))
    (desig-prop ?desig (:object ?object))
    (lisp-fun pr2-do::get-object-info ?object ?obj-info))

  ; cut
  (<- (action-desig ?desig (cut ((arm ?arm) (knife ?knife-info) (cake ?cake-info))))
    (desig-prop ?desig (:type :cut))
    (desig-prop ?desig (:arm ?arm))
    (desig-prop ?desig (:knife ?knife))
    (desig-prop ?desig (:cake ?cake))
    (lisp-fun pr2-do::get-object-info ?knife ?knife-info)
    (lisp-fun pr2-do::get-object-info ?cake ?cake-info))

  ; test
  (<- (action-desig ?desig (test ((obj-info ?obj-info))))
    (desig-prop ?desig (:type :test))))
  
