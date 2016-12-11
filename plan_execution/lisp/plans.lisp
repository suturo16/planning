(in-package :plan-execution-package)

(defun grasp-object (object-name arm)
  (let ((visible (pr2-do::is-object-in-view object-name))
        (obj-info (pr2-do::get-object-info object-name)))
    (if visible
        (progn
          (pr2-do::open-gripper arm)
          (pr2-do::move-arm-to-object obj-info arm)
          (pr2-do::close-gripper arm 0.4))
        (print "Object not in view"))))

(defun place-object (location-name object-name arm)
  (let
      ((loc-info (pr2-do::get-object-info location-name))
       (obj-info (pr2-do::get-object-info object-name)))
    (pr2-do::move-object-with-arm loc-info obj-info arm)
    (pr2-do::open-gripper arm)))
