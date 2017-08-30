(in-package :pr2-command-pool-package)

(defun close-gripper (arm &optional  (strength 50) (width 0.0))
  "Call action to close the gripper of ARM with STRENGTH."
  (action-move-gripper width arm strength))

(defun open-gripper (arm &optional (width 0.09))
  "Call action to open the gripper of ARM."
  (action-move-gripper width arm 70))

(defun disconnect-obj-from-arm (obj-info arm)
  "Call Prolog to disconnect the object of OBJ-INFO from ARM."
  (common:prolog-disconnect-frames
   (format nil "/~a_wrist_roll_link" arm)
   (format nil "/~a" (object-info-name obj-info)))

  ;; FIXME: This is a dirty hotfix, because we don't perceive regualary.
  ;; Otherwise objects disconnected from a gripper would continue to be published in the gripper frame.
  (common:prolog-connect-frames "/odom_combined"
                                (format nil "/~a" (object-info-name obj-info)))
  (common:prolog-disconnect-frames "/odom_combined"
                                   (format nil "/~a" (object-info-name obj-info))))

(defun connect-obj-with-gripper (obj-info arm)
  "Call Prolog to connect the object of OBJ-Info to ARM."
  (common:prolog-connect-frames
   (format nil "/~a_wrist_roll_link" arm)
   (format nil "/~a" (object-info-name obj-info))))

(defun move-arm-to-object (obj-info arm)
  "Call action to move ARM to the object of OBJ-INFO."
  (action-move-robot (format nil "pr2_grasp_control_~a" arm)
                     (alexandria:curry #'error-break-function +move-arm-to-object-error-limit+)
                     NIL
                     (make-param +transform+ nil "cylinder_frame"
                                 (format nil "~a ~a" (object-info-name obj-info) "base_link"))
                     (make-param +double+ T "cylinder_width" (write-to-string (object-info-width obj-info)))
                     (make-param +double+ T "cylinder_height" (write-to-string (object-info-height obj-info)))))


(defun move-object-with-arm (loc-info obj-info arm)
  "Call action to place the object of OBJ-INFO at the location of LOC-INFO.
Assume that the object is attached to ARM."
  (action-move-robot (format nil "pr2_place_control_~a" arm)
                     (alexandria:curry #'error-break-function +move-object-with-arm-error-limit+)
                     NIL
                     (make-param +transform+ NIL "target_frame"
                                 (format nil "~a ~a" (object-info-name loc-info) "base_link"))
                     (make-param +transform+ NIL "cylinder_in_gripper"
                                 (format nil "~a ~a" (object-info-name obj-info) (format nil "~a_wrist_roll_link" arm)))
                     (make-param +double+ T "cylinder_width" (write-to-string (object-info-width obj-info)))
                     (make-param +double+ T "cylinder_height" (write-to-string (object-info-height obj-info)))
                     (make-param +double+ T (format nil "~a_gripper_effort" arm) (write-to-string 50))))


;; move-n-flip constants
(alexandria:define-constant +tool-width+ 0.075)
(alexandria:define-constant +loc-radius+ 0.05)


(defun move-n-flip-object-with-arm (loc-info tool-info arm)
  "Call action to move the object on the tool of TOOL-INFO above the lcoation of LOC-INFO and flip the tool to place the obejct at the location."
  (action-move-robot (format nil "pr2_move_and_flip_~a" arm)
                     (alexandria:curry #'error-break-function +move-n-flip-object-with-arm-error-limit+)
                     NIL
                     (make-param +transform+ NIL "spatula_in_gripper"
                                 (format nil "~a ~a"
                                         (get-object-part-detail tool-info
                                                                 common:+supporting-plane-of-cake-spatula+
                                                                 common:+name-of-object+)
                                         (format nil  "~a_wrist_roll_link" arm)))
                     (make-param +transform+ NIL "goal_frame"
                                 (format nil "~a ~a" (object-info-name loc-info) "base_link"))
                     ;; (make-param +double+ T "spatula_radius" (write-to-string +tool-width+))
                     ;; (make-param +double+ T "goal_radius" (write-to-string +loc-radius+))
                     (make-param +double+ T "spatula_radius" (get-object-part-detail tool-info
                                                                                     common:+supporting-plane-of-cake-spatula+
                                                                                     common:+width-of-object+))
                     (make-param +double+ T "goal_radius" (get-object-detail loc-info common:+radius+))))

(defun get-in-base-pose ()
  "Bring PR2 into base (mantis) pose."
  (action-move-robot "pr2_upper_body_joint_control"
                     (alexandria:curry #'error-break-function +get-in-base-pose-error-limit+)
                     NIL
                     (make-param +double+ T "torso_lift_joint_goal" "0.25")
                     (make-param +double+ T "l_shoulder_pan_joint_goal" "1.23679")
                     (make-param +double+ T "l_shoulder_lift_joint_goal" "-0.247593")
                     (make-param +double+ T "l_upper_arm_roll_joint_goal" "0.614271")
                     (make-param +double+ T "l_elbow_flex_joint_goal" "-1.38094")
                     (make-param +double+ T "l_forearm_roll_joint_goal" "-4.94757")
                     (make-param +double+ T "l_wrist_flex_joint_goal" "-1.56861")
                     (make-param +double+ T "l_wrist_roll_joint_goal" "0")
                     (make-param +double+ T "r_shoulder_pan_joint_goal" "-1.23679")
                     (make-param +double+ T "r_shoulder_lift_joint_goal" "-0.247593")
                     (make-param +double+ T "r_upper_arm_roll_joint_goal" "-0.614271")
                     (make-param +double+ T "r_elbow_flex_joint_goal" "-1.38094")
                     (make-param +double+ T "r_forearm_roll_joint_goal" "4.94757")
                     (make-param +double+ T "r_wrist_flex_joint_goal" "-1.56861")
                     (make-param +double+ T "r_wrist_roll_joint_goal" "0")))



(alexandria:define-constant +blade-%+ 0.63)
;; temp constants for knife dimensions
(alexandria:define-constant +handle-length+ 0.10)
(alexandria:define-constant +handle-height+ -0.05)
(alexandria:define-constant +blade-length+ 0.172)
(alexandria:define-constant +blade-height+ 0.03)

(defun grasp-knife (knife-info arm)
  "Call action to grasp the knife of KNIFE-INFO with ARM."
  (action-move-robot (format nil "knife_grasp_~a" arm)
                     (alexandria:curry #'error-break-function +grasp-knife-error-limit+)
                     NIL
                     (make-param +transform+ NIL "target_frame" (format nil "~a ~a" (object-info-name knife-info) "base_link"))
                     (make-param +double+ T "handle_height"
                                 (common:get-object-part-detail knife-info common:+handle-of-cake-knife+ "heightOfObject"))
                                 ;;;(write-to-string +handle-height+))
                     (make-param +double+ T "handle_length"
                                 (common:get-object-part-detail knife-info common:+handle-of-cake-knife+ "lengthOfObject"))))
                                 ;;;(write-to-string +handle-length+)))))

;; adrian hat den Teller vermessen:
; durchmesser 22cm
; hoehe 3.2cm
; edge breite 2.5 cm
; edge angle 110 grad
;teller usrsprungs height: 0.016
;; teller konstanten
(alexandria:define-constant +edge-radius+ 0.15)
(alexandria:define-constant +edge-height+ 0.01)
(alexandria:define-constant +edge-width+ 0.022)
(alexandria:define-constant +edge-angle+ 1.92)

(defun grasp-plate (plate-info arm)
  (print (common:get-object-detail plate-info "radius"))
  (print (common:get-object-detail plate-info "angle"))
  "Call action to grasp the plate of PLATE-INFO with ARM."
  (action-move-robot (format nil "pr2_grasp_plate_~a" arm)
                     (alexandria:curry #'error-break-function +grasp-plate-error-limit+)
                     NIL
                     (make-param +transform+ NIL "plate_frame" (format nil "~a ~a" (object-info-name plate-info) "base_link"))
                     (make-param +double+ T "edge_radius"
                                 (common:get-object-detail plate-info "radius"))
                                 ;;;(write-to-string +edge-radius+))
                     (make-param +double+ T "edge_z"
                                 ;;;(common:get-object-detail plate-info "heightOfObject"))
                                 (write-to-string +edge-height+))
                     (make-param +double+ T "edge_depth"
                                 ;;;(common:get-object-detail plate-info "widthOfObject"))
                                 (write-to-string +edge-width+))
                     (make-param +double+ T "edge_angle"
                                 (common:get-object-detail plate-info "angle"))))
                                 ;;;(write-to-string +edge-angle+))))

;; spatula constants
; adrian hat vermessen:
; tiefe und breite, wenn man sich den pfannenheber von der seite angucktf
;4.5cm tiefe
;3.5cm breite
(alexandria:define-constant +spatula-handle-depth+ 0.045)
(alexandria:define-constant +spatula-handle-width+ 0.035)

(defun grasp-spatula (spatula-info arm)
  "Call action to grasp the spatula of SPATULA-INFO with ARM."
  (action-move-robot (format nil "pr2_grasp_fingerHandle_~a" arm)
                     (alexandria:curry #'error-break-function +grasp-spatula-error-limit+)
                     NIL
                     (make-param +transform+ NIL "handle_frame" (format nil "~a ~a" "HandleOfCakeSpatula1" "base_link"))
                     ;;;(make-param +double+ T "handle_depth" (write-to-string +spatula-handle-depth+))
                     (make-param +double+ T "handle_depth"
                                 (common:get-object-part-detail spatula-info common:+handle-of-cake-spatula+ "depthOfObject"))
                     ;;;(make-param +double+ T "handle_width" (write-to-string +spatula-handle-width+))))
                     (make-param +double+ T "handle_width"
                                 (common:get-object-part-detail spatula-info common:+handle-of-cake-spatula+ "widthOfObject"))))
                     
(defun release (arm &optional (gripper-width 0.1))
  "Call action to release the object in ARM."
  (action-move-robot (format nil "pr2_release_~a" arm)
                     (alexandria:curry #'error-break-function +release-error-limit+)
                     NIL
                     (make-param +transform+ T "start_pose" (tf-lookup->string "base_link" (format nil "~a_wrist_roll_link" arm)))
                     (make-param +double+ NIL "gripper_opening" (write-to-string gripper-width))))

(defun detach-knife-from-rack (knife-info arm)
  "Call action to move the knife of KNIFE-INFO away from the rack.
Assume that the knife is connected to ARM."
  (action-move-robot (format nil "pr2_detach_knife_~a" arm)
                     (alexandria:curry #'error-break-function +detach-knife-from-rack-error-limit+)
                     NIL
                     (make-param +transform+ NIL "knife_frame_wrist" (format nil  "~a ~a" (object-info-name knife-info)
                                                                             (format nil "~a_wrist_roll_link" arm)))
                     (make-param +transform+ T "rack_frame"  (common:tf-lookup->string "base_link" (object-info-name knife-info)))
                     (make-param 5 T "debug" "rack_frame")
                     (make-param 5 T "debug2" "rack_dist")))

(defun take-cutting-position (cake-info knife-info arm slice-width)
  "Call action to take a position ready to cut the cake of CAKE-INFO.
Assume that knife of KNIFE-INFO is used by ARM
for cutting with SLICE-WIDTH."
  (let ((handle-length (* (1- +blade-%+) (object-info-width knife-info))))
    (action-move-robot (format nil "pr2_cut_position_~a" arm)
                       (alexandria:curry #'error-break-function +take-cutting-position-error-limit+)
                       NIL
                       (make-param +transform+ NIL "cake_frame" (format nil "~a ~a" (object-info-name cake-info) "base_link"))
                       (make-param +double+ T "cake_length" (write-to-string (object-info-depth cake-info)))
                       (make-param +double+ T "cake_width" (write-to-string (object-info-width cake-info)))
                       (make-param +double+ T "cake_height" (write-to-string (object-info-height cake-info)))
                       (make-param +transform+ NIL "knife_ee_frame" (format nil "~a ~a" (object-info-name knife-info) (format nil "~a_wrist_roll_link" arm)))
                       ;; (make-param +double+ T "knife_height" (write-to-string +blade-height+))
                       (make-param +double+ T "knife_height" (common:get-object-part-detail knife-info common:+blade-of-cake-knife+ "heightOfObject"))
                       ;; (make-param +double+ T "handle_length" (write-to-string +handle-length+))
                       (make-param +double+ T "handle_length" (common:get-object-part-detail knife-info common:+handle-of-cake-knife+ "lengthOfObject"))
                       (make-param +double+ T "slice_width" (write-to-string slice-width)))))

(defun cut-cake (cake-info knife-info arm slice-width)
  "Call action to cut the cake of CAKE-INFO.
Use knife of KNIFE-INFO within the gripper of ARM
to cut pieces with SLICE-WIDTH."
  (action-move-robot (format nil "pr2_cut_~a" arm)
                     (alexandria:curry #'error-break-function +cut-cake-error-limit+)
                     NIL
                     (make-param +transform+ NIL "cake_frame" (format nil "~a ~a" (object-info-name cake-info) "base_link"))
                     (make-param +double+ T "cake_length" (write-to-string (object-info-depth cake-info)))
                     (make-param +double+ T "cake_width" (write-to-string (object-info-width cake-info)))
                     (make-param +double+ T "cake_height" (write-to-string (object-info-height cake-info)))
                     (make-param +transform+ NIL "knife_in_gripper"
                                 (format nil "~a ~a" (object-info-name knife-info) (format nil "~a_wrist_roll_link" arm)))
                     ;; (make-param +double+ T "knife_height" (write-to-string +blade-height+))
                     (make-param +double+ T "knife_height"
                                 (common:get-object-part-detail knife-info common:+blade-of-cake-knife+ "heightOfObject"))
                     (make-param +double+ T "slice_width" (write-to-string slice-width))))


(defun move-slice-aside (knife-info cake-info target-info arm)
  (action-move-robot (format nil "pr2_move_cake_~a" arm)
                     (alexandria:curry #'error-break-function +move-slice-aside-error-limit+)
                     NIL
                     (make-param +transform+ NIL "knife_in_gripper" (format nil "~a ~a_wrist_roll_link" (object-info-name knife-info) arm))
                     (make-param +transform+ NIL "cake" (format nil "~a base_link" (object-info-name cake-info)))
                     (make-param +transform+ NIL "plate" (format nil "~a base_link" (object-info-name target-info)))))


(defun look-at (obj-info)
  "Call action to look at the position of the object of OBJ-INFO."
  (action-move-robot (format nil "pr2_look_at")
                     (alexandria:curry #'error-break-function +look-at-error-limit+)
                     NIL
                     (make-param +transform+ NIL "focal_point" (format nil "~a ~a" (object-info-name obj-info) "base_link"))))
