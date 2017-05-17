(in-package :plan-execution-package)

(defmacro with-logging-node (node-name &body body)
  `(let ((log-node (beliefstate:start-node ,node-name nil)))
    (unwind-protect
          (progn ,@body)
       (beliefstate:stop-node log-node :success ,T))))


(defun seen-since (obj-info)
  "Check if object described by OBJ-INFO is still at the last known location."
  (let ((name (common:object-info-name obj-info))
        (frame-id (common:object-info-frame obj-info))
        (timestamp (common:object-info-timestamp obj-info)))
    (if (common:prolog-seen-since name frame-id timestamp)
        T
        (error 'common:seen-since-failure))))


(defun check-object-location (obj-info)
  "Return T if the object of OBJ-INFO is still at the same location."
  (when obj-info
    (pr2-do:get-in-base-pose)
    (cpl:with-retry-counters ((seen-since 3) (perception 1))
      (cpl:with-failure-handling
          ((common:seen-since-failure (e)
             (declare (ignore e))
             (cpl:do-retry seen-since
               (cpl:retry))
             (ros-warn (check-object-location) "Returning T although seen-since didn't work.")
             (return T))
           (common:perception-pipeline-failure (e)
             (cpl:do-retry perception
               (ros-warn (check-object-location) "~a" e)
               (cpl:retry))))
        (pr2-do:look-at obj-info)
        (common:run-pipeline (common:object-info-name obj-info))
        (when (seen-since obj-info)
          T)))))


(cpl:def-cram-function grasp (obj-info arm)
  "Grasp the object described by OBJECT-INFO with ARM.\

OBJ-INFO (obj-info): Description of the target object.
ARM (string): Which arm to use. Use one of the constants defined in planning-common."
  (ros-info (grasp) "Starting to grasp object ~a with arm ~a."
            (common:object-info-name obj-info)
            arm)
  (if (check-object-location obj-info)
      ;; if object found, grasp it
      (with-logging-node "GRASP-OBJECT"
        (beliefstate::annotate-resource "objectInfo" (common:object-info-name obj-info) "knowrob")
        (beliefstate::annotate-resource "arm" arm "knowrob")
        ;; grasp it
        (seq
          (cpl:with-retry-counters ((timeouts 1))
            (cpl:with-failure-handling
                ((common:action-timeout (e)
                   (cpl:do-retry timeouts
                     (ros-warn (grasp) "Grasping went-wrong: ~a" e)
                     (cpl:retry))))
              (alexandria:switch ((common:object-info-name obj-info) :test #'equal)
                ("Knife" (grasp-knife obj-info arm))
                ("Plate" (grasp-plate obj-info arm))
                ("Cylinder" (grasp-object obj-info arm)))))
          (pr2-do:connect-obj-with-gripper obj-info arm)
          (ros-info (grasp) "Connected object ~a with arm ~a."
                    (common:object-info-name obj-info)
                    arm)))
      ;; else complain
      (ros-error (grasp) "Object ~a not found" (common:object-info-name obj-info))))
  

(defun grasp-knife (knife-info arm)
  "Grasp the knife described by KNIFE-INFO with ARM."
  (pr2-do:grasp-knife knife-info arm)
  (ros-info (grasp knife) "Close gripper.")
  (pr2-do:close-gripper arm 100))


(defun grasp-plate (plate-info arm)
  "Grasp the plate described by PLATE-INFO with ARM."
  (ros-info (grasp plate) "Open gripper")
  (pr2-do:open-gripper arm)
  (ros-info (grasp plate) "Move arm to object ~a." (common:object-info-name plate-info))
  (pr2-do:grasp-plate plate-info arm)
  (ros-info (grasp plate) "Close gripper.")
  (pr2-do:close-gripper arm 50)
  (ros-info (grasp plate) "Done."))


(defun grasp-object (obj-info arm)
  "Grasp the object described by OBJECT-INFO with ARM."
  (ros-info (grasp object) "Open gripper")
  (pr2-do:open-gripper arm)
  (ros-info (grasp object) "Move arm to object ~a." (common:object-info-name obj-info))
  (pr2-do:move-arm-to-object obj-info arm)
  (ros-info (grasp object) "Close gripper.")
  (pr2-do:close-gripper arm 50)
  (ros-info (grasp object) "Done."))


(cpl:def-cram-function place-object (obj-info loc-info arm)
  "Place object described by OBJECT-INFO at location described by LOCATION-INFO with ARM.
Assuming the object is placed on ARM's gripper.

OBJ-INFO (common:obj-info): Description of the target object.
LOC-INFO (common:obj-info): Description of target location.
ARM (string): Which arm to use. Use one of the constants defined in planning-common."
  (with-logging-node "PLACE-OBJECT"
    (beliefstate::annotate-resource "objectInfo" (common:object-info-name obj-info) "knowrob")
    (beliefstate::annotate-resource "locationInfo" loc-info "knowrob")
    (beliefstate::annotate-resource "arm" arm "knowrob")
    (ros-info (place-object) "Move object with arm.")
    (pr2-do:move-object-with-arm loc-info obj-info arm)
    (ros-info (place-object) "Open gripper.")
    (pr2-do:open-gripper arm)
    (ros-info (place-object) "Done.")))


(cpl:def-cram-function detach-object-from-rack (obj-info arm)
  "Detach object described by OBJECT-INFO from the rack with ARM.
Assume object is attached to the rack.

OBJ-INFO (obj-info): Description of the target object.
ARM (string): Which arm to use. Use one of the constants defined in planning-common."
  ;; this doesn't work, I think the axes are wrong since bot moves his torso down
  (ros-info (detach-object-from-rack) "Detaching.")
  (pr2-do:detach-knife-from-rack obj-info arm)
  (ros-info (detach-object-from-rack) "Done."))
  

(cpl:def-cram-function cut-object (arm knife-info cake-info)
  "Cut object described by CAKE-INFO with knife described by KNIFE-INFO using ARM.
Assume the knife is in the gripper of ARM.

CAKE-INFO (obj-info): Description of the target object.
KNIFE-INFO (obj-info): Description of the cutting tool.
ARM (string): Which arm to use. Use one of the constants defined in planning-common."
 (if (check-object-location cake-info)
      ;; if object is found, cut it
      (with-logging-node "CUT-OBJECT"
        (beliefstate::annotate-resource "knifeInfo" (common:object-info-name knife-info) "knowrob")
        (beliefstate::annotate-resource "cakeInfo" (common:object-info-name cake-info) "knowrob")
        (beliefstate::annotate-resource "arm" arm "knowrob")
        (common:service-connect-frames "/odom_combined" "/Box")
        
        (ros-info (cut-object) "Getting into cutting position.")
        (cpl:with-retry-counters ((timeouts 1))
          (cpl:with-failure-handling
              ((common:action-timeout (e)
                 (cpl:do-retry timeouts
                   (ros-warn (cut take-cutting-position) "Taking cutting position went wrong: ~a" e)
                   (cpl:retry))))
            (pr2-do:take-cutting-position cake-info knife-info arm 0.01)))

        (ros-info (cut-object) "Cutting.")
        (cpl:with-retry-counters ((timeouts 1))
          (cpl:with-failure-handling
              ((common:action-timeout (e)
                 (cpl:do-retry timeouts
                   (ros-warn (cut cutting) "Cutting went wrong: ~a" e)
                   (cpl:retry))))
            (pr2-do:cut-cake cake-info knife-info arm 0.01)))
        
        ; TODO(cpo): get cake-piece-info
        ; (pr2-do::push-aside cake-info cake-piece-info)
        (ros-info (cut-object) "Done."))
      ;; else complain
      (ros-error (cut-object) "Cannot find object '~a', which I am supposed to cut."
                 (common:object-info-name cake-info))))


(defun ms2-grasp-knife (knife-info arm)
  (ros-info (ms2-grasp-knife) "connect Knife frame to ododm.")
  (common:service-connect-frames "/odom_combined"  "/Knife")
  (ros-info (ms2-grasp-knife) "grasp the knife.")
  (pr2-do:grasp-knife knife-info arm)
  (ros-info (ms2-grasp-knife) "close-gripper")
  (pr2-do:close-gripper arm 100)
  ;; (pr2-do::detach-knife-from-rack (pr2-do::get-object-info "Knife") pr2-do::+right-arm+)
  (ros-info (ms2-grasp-knife) "reconnect frames from odom to arm")
  (common:prolog-disconnect-frames "/odom_combined" "/Knife")
  (common:service-connect-frames "/r_wrist_roll_link" "/Knife")
  (pr2-do:get-in-base-pose)
  ;;TODO: test detach, add it in here.
  )


(defun ms2-cut-cake (cake-info knife-info arm)
  (common:service-connect-frames "/odom_combined" "/Box")
  (ros-info "ms2-cut-object" "Getting into cutting position.")
  (pr2-do:take-cutting-position cake-info knife-info arm 0.01)
  (ros-info "ms2-cut-object" "Cutting.")
  (pr2-do:cut-cake cake-info knife-info arm 0.01)
  (ros-info "ms2-cut-cake" "Done."))


(defun ms2-full-demo ()
  (ms2-grasp-knife (common:get-object-info "Knife") common:+right-arm+ )
  (pr2-do:get-in-base-pose)
  (ms2-cut-cake (common:get-object-info "Box") (common:get-object-info "Knife") common:+right-arm+))
