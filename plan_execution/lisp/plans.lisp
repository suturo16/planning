(in-package :plan-execution-package)

(defmacro with-logging-node (node-name &body body)
  `(let ((log-node (beliefstate:start-node ,node-name nil)))
    (unwind-protect
          (progn ,@body)
       (beliefstate:stop-node log-node :success ,T))))

(cram-language:def-cram-function grasp (obj-info arm)
  (print "grasp:: Check for object.")
  (if (pr2-do::check-object-location obj-info)
      ;; grasp it
      (with-logging-node "GRASP-OBJECT"
        (beliefstate::annotate-resource "objectInfo" (pr2-do::object-info-name obj-info) "knowrob")
        (beliefstate::annotate-resource "arm" arm "knowrob")
        (alexandria:switch ((pr2-do::object-info-name obj-info))
          ("knife" (pr2-do::grasp-knife obj-info arm))
          ("cylinder" (grasp-object obj-info arm)))
        (format t "something good has happened.............."))
      ;; else complain
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
  (with-logging-node "PLACE-OBJECT"
    (beliefstate::annotate-resource "objectInfo" (pr2-do::object-info-name obj-info) "knowrob")
    (beliefstate::annotate-resource "locationInfo" loc-info "knowrob")
    (beliefstate::annotate-resource "arm" arm "knowrob")
    (ros-info "place-object" "Move object with arm.")
    (pr2-do::move-object-with-arm loc-info obj-info arm)
    (ros-info "place-object" "Open gripper.")
    (pr2-do::open-gripper arm)
    (ros-info "place-object" "Done.")))

(cram-language:def-cram-function cut-object (arm knife-info cake-info &optional (slice-width 0.01))
  "Cut obj with knife in arm."
  (if (pr2-do::check-object-location cake-info)
      ;; if object is found, cut it
      (with-logging-node "CUT-OBJECT"
        (beliefstate::annotate-resource "knifeInfo" (pr2-do::object-info-name knife-info) "knowrob")
        (beliefstate::annotate-resource "cakeInfo" (pr2-do::object-info-name cake-info) "knowrob")
        (beliefstate::annotate-resource "arm" arm "knowrob")
        (ros-info "cut-object" "Getting into cutting position.")
        (pr2-do::take-cutting-position knife-info cake-info arm slice-width)
        (ros-info "cut-object" "Cutting.")
        (pr2-do::cut-cake knife-info cake-info arm slice-width)
        ;; TODO(cpo): get cake-piece-info
        ;; (pr2-do::push-aside cake-info cake-piece-info)
        (ros-info "cut-object" "Done."))
      ;; else complain
      (ros-error "cut-object" "Cannot find object which I am supposed to cut.")))
