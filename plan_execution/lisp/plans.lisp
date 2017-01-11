(in-package :plan-execution-package)

(defun grasp-object (object-name arm)
  (print "grasp object:: check is object is visible")
  (let ((visible (pr2-do::is-object-in-view object-name))
        (obj-info (pr2-do::get-object-info object-name)))
    (if visible
        (progn
          (print "grasp object:: object visible. try open gripper")
          (pr2-do::open-gripper arm)
          (print "grasp object:: move arm to object")
          (pr2-do::move-arm-to-object obj-info arm)
          (print "grasp object:: close gripper")
          (pr2-do::close-gripper arm 50)
          (print "grasp object:: done"))
        (print "Object not in view"))))

(defun place-object (location-name object-name arm)
  (let
      ((loc-info (pr2-do::get-object-info location-name))
       (obj-info (pr2-do::get-object-info object-name)))
    (print "place object:: move object with arm")
    (pr2-do::move-object-with-arm loc-info obj-info arm)
    (print "place object:: open gripper ")
    (pr2-do::open-gripper arm)
    (print "place object:: done")))
