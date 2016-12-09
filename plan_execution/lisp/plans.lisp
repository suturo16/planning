(in-package :plan-execution-package)

(defun grasp-object (object-id arm)
  (let
      ((visible (pr2-do::is-object-in-view object-id))
       (obj-loc (pr2-do::get-object-location object-id)))
    (if visible
        (progn
          (pr2-do::move-arm-to-object obj-loc arm)
          (pr2-do::close-gripper arm))
        (print "Object not in view"))))

(defun place-object (location-id arm)
  (let
      ((location (pr2-do::get-drop-location location-id)))
    (pr2-do::move-object-with-arm location  arm)
    (pr2-do::open-gripper arm)))
