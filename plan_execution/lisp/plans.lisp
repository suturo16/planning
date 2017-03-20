 (in-package :plan-execution-package)

(defmacro with-logging-node (node-name &body body)
  `(let ((log-node (beliefstate:start-node ,node-name nil)))
    (unwind-protect
          (progn ,@body)
       (beliefstate:stop-node log-node :success ,T))))

(cram-language:def-cram-function grasp (obj-info arm)
  (ros-info "grasp" "Starting to grasp object ~a with arm ~a."
            (pr2-do::object-info-name obj-info)
            arm)
  ;;(if (pr2-do::check-object-location obj-info)
      ;; grasp it
      (with-logging-node "GRASP-OBJECT"
        (beliefstate::annotate-resource "objectInfo" (pr2-do::object-info-name obj-info) "knowrob")
        (beliefstate::annotate-resource "arm" arm "knowrob")
        ;; grasp it
        (seq
          (alexandria:switch ((pr2-do::object-info-name obj-info) :test #'equal)
            ("Knife" (pr2-do::grasp-knife obj-info arm))
            ("Cylinder" (grasp-object obj-info arm)))
          (pr2-do::connect-obj-with-gripper obj-info arm)
          (ros-info "grasp" "Connected object ~a with arm ~a."
                    (pr2-do::object-info-name obj-info)
                    arm)))
      ;; else complain
      ;;(ros-error "grasp" "Object ~a not found" (pr2-do::object-info-name obj-info)))
  )
  


(defun grasp-knife (knife-info arm)
;; (pr2-do::grasp-knife knife-info arm)
;; (pr2-do::close-gripper arm 100)
  (ms2-grasp-knife knife-info arm))

(defun ms2-grasp-knife (knife-info arm)
  (ros-info "ms2-grasp-knife" "connect Knife frame to ododm.")
  (pr2-do::service-connect-frames "/odom_combined"  "/Knife")
  (ros-info "ms2-grasp-knife" "grasp the knife.")
  (pr2-do::grasp-knife knife-info arm)
  (ros-info "ms2-grasp-knife" "close-gripper")
  (pr2-do::close-gripper arm 100)
  (ros-info "ms2-grasp-knife" "reconnect frames from odom to arm")
 
 ;; (pr2-do::detach-knife-from-rack (pr2-do::get-object-info "Knife") pr2-do::+right-arm+)
  (pr2-do::prolog-disconnect-frames "/odom_combined" "/Knife")
  (pr2-do::service-connect-frames "/r_wrist_roll_link" "/Knife")
  (pr2-do::get-in-base-pose)
  ;;TODO: test detach, add it in here. 
  )

(defun grasp-object (obj-info arm)
  (ros-info "grasp-object" "Open gripper")
  (pr2-do::open-gripper arm)
  (ros-info "grasp-object" "Move arm to object ~a." (pr2-do::object-info-name obj-info))
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



(cram-language:def-cram-function detach-object-from-rack (obj-info arm)
  ;; this doesn't work, I think the axes are wrong since bot moves his torso down
  (pr2-do::detach-knife-from-rack obj-info arm)
  ;;(pr2-do::get-in-base-pose)
  )
  

(cram-language:def-cram-function cut-object (arm knife-info cake-info)
  "Cut obj with knife in arm."
 ;;(if (pr2-do::check-object-location cake-info)
      ;; if object is found, cut it
      (with-logging-node "CUT-OBJECT"
        (beliefstate::annotate-resource "knifeInfo" (pr2-do::object-info-name knife-info) "knowrob")
        (beliefstate::annotate-resource "cakeInfo" (pr2-do::object-info-name cake-info) "knowrob")
        (beliefstate::annotate-resource "arm" arm "knowrob")
        (pr2-do::service-connect-frames "/odom_combined" "/Box")
        (ros-info "-cut-object" "Getting into cutting position.")
        (pr2-do::take-cutting-position cake-info knife-info arm 0.01)
        (ros-info "cut-object" "Cutting.")
        (pr2-do::cut-cake cake-info knife-info arm 0.01)
        ; TODO(cpo): get cake-piece-info
        ; (pr2-do::push-aside cake-info cake-piece-info)
        (ros-info "cut-object" "Done."))
      ;; else complain
      ;;(ros-error "cut-object" "Cannot find object '~a', which I am supposed to cut."
                 ;;(pr2-do::object-info-name cake-info)))
  )

(defun ms2-cut-cake (cake-info knife-info arm)
  (pr2-do::service-connect-frames "/odom_combined" "/Box")
  (ros-info "ms2-cut-object" "Getting into cutting position.")
  (pr2-do::take-cutting-position cake-info knife-info arm 0.01)
  (ros-info "ms2-cut-object" "Cutting.")
  (pr2-do::cut-cake cake-info knife-info arm 0.01)
  (ros-info "ms2-cut-cake" "Done."))

(defun ms2-full-demo ()
  (plan-execution-package::ms2-grasp-knife (common:get-object-info "Knife") pr2-do::+right-arm+ )
  (pr2-do::get-in-base-pose)
  (plan-execution-package::ms2-cut-cake (pr2-do::get-object-info "Box") (pr2-do::get-object-info "Knife") pr2-do::+right-arm+))
