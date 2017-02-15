(in-package :plan-execution-package)

(cram-language:def-cram-function grasp (obj-info arm)
  (print "grasp:: Check for object.")
  (if (pr2-do::check-object-location obj-info)
      ; grasp it
      (alexandria:switch ((pr2-do::object-info-name obj-info))
        ("knife" (pr2-do::grasp-knife obj-info arm))
        ("cylinder" (grasp-object obj-info arm)))
      ; else complain
      (ros-error "grasp" "Object not found")))

(defun grasp-object (obj-info arm)
  (ros-info "grasp-object" "Open gripper")
  (pr2-do::open-gripper arm)
  (ros-info "grasp-object" "Move arm to object.")
  (pr2-do::move-arm-to-object obj-info arm)
  (ros-info "grasp-object" "Close gripper.")
  (pr2-do::close-gripper arm 50)
  (ros-info "grasp-object" "Done."))

(cram-language:def-cram-function place-object (obj-info loc-info arm)
  (ros-info "place-object" "Move object with arm.")
  (pr2-do::move-object-with-arm loc-info obj-info arm)
  (ros-info "place-object" "Open gripper.")
  (pr2-do::open-gripper arm)
  (ros-info "place-object" "Done."))

(cram-language:def-cram-function cut-object (arm knife-info cake-info)
  "Cut obj with knife in arm."
  (if (pr2-do::check-object-location cake-info)
      ; if object is found, cut it
      (progn
        (ros-info "cut-object" "Getting into cutting position.")
        (pr2-do::take-cutting-position knife-info cake-info arm)
        (ros-info "cut-object" "Cutting.")
        (pr2-do::cut-cake knife-info cake-info arm)
        ; TODO(cpo): get cake-piece-info
        ; (pr2-do::push-aside cake-info cake-piece-info)
        (ros-info "cut-object" "Done."))
      ; else complain
      (ros-error "cut-object" "Cannot find object which I am supposed to cut.")))
