(in-package :plan-execution-package)

(cram-language:def-cram-function grasp-object (obj-info arm)
  (print "grasp object:: check is object is visible")
  (let ((visible (pr2-do::is-object-in-view (pr2-do::object-info-name obj-info))))
    (if visible
        (progn
          (print "grasp object:: object visible. open gripper")
          (pr2-do::open-gripper arm)
          (print "grasp object:: move arm to object")
          (pr2-do::move-arm-to-object obj-info arm)
          (print "grasp object:: close gripper")
          (pr2-do::close-gripper arm 50)
          (print "grasp object:: done"))
        (print "Object not in view"))))

(cram-language:def-cram-function place-object (obj-info loc-info arm)
  (print "place object:: move object with arm")
  (pr2-do::move-object-with-arm loc-info obj-info arm)
  (print "place object:: open gripper ")
  (pr2-do::open-gripper arm)
  (print "place object:: done"))

(cram-language:def-cram-function cut-object (arm knife-info obj-info)
  (if (pr2-do::is-object-in-view obj-info)
      (progn 
        (pr2-do::slice obj-info arm)
        (pr2-do::push-aside obj-info arm))
      (print "cannot see object which I am supposed to cut")))
