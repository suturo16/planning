(in-package :plan-execution-package)

(defun pick-up-object (object-id arm)
  (let
      ((visible (pr2-do::is-object-in-view object-id))
       (obj-loc (pr2-do::get-object-location object-id)))
    (if visible
        (progn
          (pr2-do::grasp-object  obj-loc arm)
          (pr2-do::lift-object arm))
        (print "Object not in view"))))

(defun put-down-object (location-id arm)
  (let
      ((location (pr2-do::get-drop-location location-id)))
    (pr2-do::put-object-down-to location  arm)
    (pr2-do::open-gripper arm)
    (pr2-do::get-in-base-pose)))
