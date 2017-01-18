(in-package :pr2-command-pool-package)

(def-fact-group move-robot-actions (action-desig)
  ; grasp
  (<- (action-desig ?desig (grasp ?arm ?obj-info))
    (desig-prop ?desig (:type :grasp))
    (desig-prop ?desig (:arm ?arm))
    (desig-prop ?desig (:object ?object))
    (lisp-fun get-object-info ?object ?obj-info))

  ; place
  (<- (action-desig ?desig (place ?arm ?target ?obj-info))
    (desig-prop ?desig (:type :place))
    (desig-prop ?desig (:arm ?arm))
    (desig-prop ?desig (:target ?target))
    (desig-prop ?desig (:object ?object))
    (lisp-fun get-object-info ?object ?obj-info))

  (<- (action-desig ?desig (test ?obj-info))
    (desig-prop ?desig (:type :test))
    (desig-prop ?desig (:object ?object))
    (lisp-fun get-object-info ?object ?obj-info)))
