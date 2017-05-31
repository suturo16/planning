(in-package :pr2-command-pool-package)

(defun close-gripper (arm &optional  (strength 50))
  "Call action to close the gripper of ARM with STRENGTH."
  (action-move-gripper 0.0 arm strength))

(defun open-gripper (arm &optional (width 0.09))
  "Call action to open the gripper of ARM."
  (action-move-gripper width arm 70))

(defun disconnect-obj-from-arm (obj-info arm)
  "Call Prolog to disconnect the object of OBJ-INFO from ARM."
  (prolog-disconnect-frames
   (format nil "/~a_wrist_roll_link" arm)
   (format nil "/~a" (object-info-name obj-info))))

(defun connect-obj-with-gripper (obj-info arm)
  "Call Prolog to connect the object of OBJ-Info to ARM."
  (service-connect-frames
   (format nil "/~a_wrist_roll_link" arm)
   (format nil "/~a" (object-info-name obj-info))))

(defun move-arm-to-object (obj-info arm)
  "Call action to move ARM to the object of OBJ-INFO."
  (let ((arm-str (if (string= arm +left-arm+) "left" "right")))
    (action-move-robot (format nil "pr2_upper_body_~a_arm" arm-str)
                       (format nil "pr2_grasp_control_~a" arm)
                       (lambda (v) (< v 0.01))
                       (make-param +transform+ nil "object_frame"
                                   (format nil "~a ~a" (object-info-name obj-info) "base_link"))
                       (make-param +double+ T "object_width" (write-to-string (object-info-width obj-info)))
                       (make-param +double+ T "object_height" (write-to-string (object-info-height obj-info))))))

(defun move-object-with-arm (loc-info obj-info arm)
  "Call action to place the object of OBJ-INFO at the location of LOC-INFO.
Assume that the object is attached to ARM."
  (let ((arm-str (if (string= arm +left-arm+) "left" "right")))
    (action-move-robot (format nil "pr2_upper_body_~a_arm" arm-str)
                       (format nil "pr2_place_control_~a" arm)
                       (lambda (v) (< v 0.01))
                       (make-param +transform+ NIL "location_frame"
                                   (format nil "~a ~a" (object-info-name loc-info) "base_link"))
                       (make-param +transform+ NIL "object_frame"
                                   (format nil "~a ~a" (object-info-name obj-info) (format nil "~a_wrist_roll_link" arm)))
                       (make-param +double+ T "object_width" (write-to-string (object-info-width obj-info)))
                       (make-param +double+ T "object_height" (write-to-string (object-info-height obj-info)))
                       (make-param +double+ T (format nil "~a_gripper_effort" arm) (write-to-string 50)))))


;; move-n-flip constants
(alexandria:define-constant +tool-width+ 0.075)
(alexandria:define-constant +loc-radius+ 0.2)

(defun move-n-flip-object-with-arm (loc-info tool-info arm)
  "Call action to move the object on the tool of TOOL-INFO above the lcoation of LOC-INFO and flip the tool to place the obejct at the location."
  (let ((arm-str (if (string= arm +left-arm+) "left" "right")))
    (action-move-robot (format nil "pr2_upper_body_~a_arm" arm-str)
                       (format nil "pr2_place_control_~a" arm)
                       (lambda (v) (< v 0.01))
                       (make-param +transform+ NIL "tool_frame"
                                   (format nil "~a ~a" (object-info-name tool-info) (format nil  "~a_wrist_roll_link" arm)))
                       (make-param +transform+ NIL "location_frame"
                                   (format nil "~a ~a" (object-info-name loc-info) "base_link"))
                       (make-param +double+ T "tool_width" +tool-width+)
                       (make-param +double+ T "loc-radius" +loc-radius+))))

(defun get-in-base-pose ()
  "Bring PR2 into base (mantis) pose."
  (action-move-robot "pr2_upper_body" "pr2_upper_body_joint_control" (lambda (v) (< v 0.05))
                     (make-param +double+ T "torso_lift_joint" "0.25")
                     (make-param +double+ T "l_shoulder_pan_joint" "1.23679")
                     (make-param +double+ T "l_shoulder_lift_joint" "-0.247593")
                     (make-param +double+ T "l_upper_arm_roll_joint" "0.614271")
                     (make-param +double+ T "l_elbow_flex_joint" "-1.38094")
                     (make-param +double+ T "l_forearm_roll_joint" "-4.94757")
                     (make-param +double+ T "l_wrist_flex_joint" "-1.56861")
                     (make-param +double+ T "l_wrist_roll_joint" "0")
                     (make-param +double+ T "r_shoulder_pan_joint" "-1.23679")
                     (make-param +double+ T "r_shoulder_lift_joint" "-0.247593")
                     (make-param +double+ T "r_upper_arm_roll_joint" "-0.614271")
                     (make-param +double+ T "r_elbow_flex_joint" "-1.38094")
                     (make-param +double+ T "r_forearm_roll_joint" "4.94757")
                     (make-param +double+ T "r_wrist_flex_joint" "-1.56861")
                     (make-param +double+ T "r_wrist_roll_joint" "0")))



(alexandria:define-constant +blade-%+ 0.63)
;; temp constants for knife dimensions
(alexandria:define-constant +handle-length+ 0.105)
(alexandria:define-constant +handle-height+ 0.04)
(alexandria:define-constant +blade-length+ 0.172)
(alexandria:define-constant +blade-height+ 0.057)


(defun grasp-knife (knife-info arm)
  "Call action to grasp the knife of KNIFE-INFO with ARM."
  (let* ((arm-str (if (string= arm +left-arm+) "left" "right"))
         (grip-length (* (1- +blade-%+) (object-info-width knife-info)))
         (blade-height (object-info-height knife-info)))
    (action-move-robot (format nil "pr2_upper_body_~a_arm" arm-str)
                       "knife_grasp"
                       (lambda (v) (< v 0.025))
                       (make-param +transform+ NIL "knife_frame" (format nil  "~a ~a" (object-info-name knife-info) "base_link"))
                       (make-param +double+ T "blade_height" (write-to-string +blade-height+))
                       (make-param +double+ T "handle_length" (write-to-string +handle-length+)))))

;; adrian hat den Teller vermessen:
; durchmesser 22cm
; hoehe 3.2cm
; edge breite 2.5 cm
; edge angle 110 grad
;; teller konstanten
(alexandria:define-constant +edge-radius+ 0.11)
(alexandria:define-constant +edge-height+ 0.016)
(alexandria:define-constant +edge-width+ 0.025)
(alexandria:define-constant +edge-angle+ 1.92)

(defun grasp-plate (plate-info arm)
  "Call action to grasp the plate of PLATE-INFO with ARM."
  (let* ((arm-str (if (string= arm +left-arm+) "left" "right")))
    (action-move-robot (format nil "pr2_upper_body_~a_arm" arm-str)
                       (format nil "pr2_grasp_plate_~a" arm)
                       (lambda (v) (< v 0.025))
                       (make-param +transform+ NIL "edge_center_frame" (format nil "~a ~a" (object-info-name plate-info) "base_link"))
                       (make-param +double+ T "edge_radius" (write-to-string +edge-radius+))
                       (make-param +double+ T "edge_height" (write-to-string +edge-height+))
                       (make-param +double+ T "edge_width" (write-to-string +edge-width+))
                       (make-param +double+ T "edge_angle" (write-to-string +edge-angle+)))))

;; spatula constants
; adrian hat vermessen:
; tiefe und breite, wenn man sich den pfannenheber von der seite anguckt
;4.5cm tiefe
;3.5cm breite
(alexandria:define-constant +spatula-handle-depth+ 0.045)
(alexandria:define-constant +spatula-handle-width+ 0.035)

(defun grasp-spatula (spatula-info arm)
  "Call action to grasp the spatula of SPATULA-INFO with ARM."
  (let* ((arm-str (if (string= arm +left-arm+) "left" "right")))
    (action-move-robot (format nil "pr2_upper_body_~a_arm" arm-str)
                       (format nil "pr2_grasp_fingerHandle_~a" arm)
                       (lambda (v) (< v 0.025))
                       (make-param +transform+ NIL "spatula_handle_tf" (format nil "~a ~a" "spatula_handle" "base_link"))
                       (make-param +double+ T "handle_depth" (write-to-string +spatula-handle-depth+))
                       (make-param +double+ T "handle_width" (write-to-string +spatula-handle-width+)))))

(defun release (arm &optional (gripper-width 0.1))
  "Call action to release the object in ARM."
  (let* ((arm-str (if (string= arm +left-arm+) "left" "right")))
    (action-move-robot (format nil "pr2_upper_body_~a_arm" arm-str)
                       (format nil "pr2_release_~a" arm)
                       (lambda (v) (< v 0.03))
                       (make-param +transform+ T "gripper_frame" (tf-lookup->string "base_link" (format nil "~a_wrist_roll_link" arm)))
                       (make-param +double+ NIL "gripper-width" gripper-width))))

(defun detach-knife-from-rack (knife-info arm)
  "Call action to move the knife of KNIFE-INFO away from the rack.
Assume that the knife is connected to ARM."
  (let* ((arm-str (if (string= arm +left-arm+) "left" "right")))
    (action-move-robot (format nil "pr2_upper_body_~a_arm" arm-str)
                       (format nil "pr2_detach_knife_~a" arm)
                       (lambda (v) (< v 0.000001))
                       (make-param +transform+ NIL "knife_frame" (format nil  "~a ~a" (object-info-name knife-info)
                                                                 (format nil "~a_wrist_roll_link" arm)))
                       (make-param +transform+ T "original_knife_tf" (tf-lookup->string "base_link" (object-info-name knife-info))))))

(defun take-cutting-position (cake-info knife-info arm slice-width)
  "Call action to take a position ready to cut the cake of CAKE-INFO.
Assume that knife of KNIFE-INFO is used by ARM
for cutting with SLICE-WIDTH."
  (let ((arm-str (if (string= arm +left-arm+) "left" "right"))
        (handle-length (* (1- +blade-%+) (object-info-width knife-info))))
    (action-move-robot (format nil "pr2_upper_body_~a_arm" arm-str)
                       (format nil "pr2_cut_position_~a" arm)
                       (lambda (v) (< v 0.01))
                       (make-param +transform+ NIL "cake_tf" (format nil "~a ~a" (object-info-name cake-info) "base_link"))
                       (make-param +double+ T "cake_length_x" (write-to-string (object-info-depth cake-info)))
                       (make-param +double+ T "cake_width_y" (write-to-string (object-info-width cake-info)))
                       (make-param +double+ T "cake_height_z" (write-to-string (object-info-height cake-info)))
                       (make-param +transform+ NIL "knife_tf" (format nil "~a ~a" (object-info-name knife-info) (format nil "~a_wrist_roll_link" arm)))
                       (make-param +double+ T "blade_height" (write-to-string +blade-height+))  ; (write-to-string (object-info-height knife-info)))
                       (make-param +double+ T "handle_length" (write-to-string +handle-length+))
                       (make-param +double+ T "slice_width" (write-to-string slice-width)))))

(defun cut-cake (cake-info knife-info arm slice-width)
  "Call action to cut the cake of CAKE-INFO.
Use knife of KNIFE-INFO within the gripper of ARM
to cut pieces with SLICE-WIDTH."
  (let ((arm-str (if (string= arm +left-arm+) "left" "right"))
        (handle-length (* (1- +blade-%+) (object-info-width knife-info))))
    (action-move-robot (format nil "pr2_upper_body_~a_arm" arm-str)
                       (format nil "pr2_cut_~a" arm)
                       (lambda (v) (< v 0.01))
                       (make-param +transform+ NIL "cake_tf" (format nil "~a ~a" (object-info-name cake-info) "base_link"))
                       (make-param +double+ T "cake_length_x" (write-to-string (object-info-depth cake-info)))
                       (make-param +double+ T "cake_width_y" (write-to-string (object-info-width cake-info)))
                       (make-param +double+ T "cake_height_z" (write-to-string (object-info-height cake-info)))
                       (make-param +transform+ NIL "knife_tf" (format nil "~a ~a" (object-info-name knife-info) (format nil "~a_wrist_roll_link" arm)))
                       (make-param +double+ T "blade_height" (write-to-string +blade-height+)) ; (write-to-string (object-info-height knife-info)))
                       (make-param +double+ T "handle_length" (write-to-string +handle-length+))
                       (make-param +double+ T "slice_width" (write-to-string slice-width)))))


(defun move-slice-aside (knife-info cake-info target-info arm)
  (let ((arm-str (if (string= arm +left-arm+) "left" "right")))
    (action-move-robot (format nil "pr2_move_slice_~a" arm)
                       (format nil "pr2_upper_body_~a_arm" arm-str)
                       (make-param +transform+ NIL "knife_tf" (format nil "~a ~a_wrist_roll_link" (object-info-name knife-info) arm))
                       (make-param +transform+ NIL "cake_tf" (format nil "~a base_link" (object-info-name cake-info)))
                       (make-param +transform+ NIL "target_tf" (format nil "~a base_link" (object-info-name target-info))))))


(defun look-at (obj-info)
  "Call action to look at the position of the object of OBJ-INFO."
    (action-move-robot (format nil "pr2_lookAt_joints")
                       (format nil "pr2_look_at")
                       (lambda (v) (< v 0.01))
                       (make-param +transform+ NIL "obj_frame" (format nil "~a ~a" (object-info-name obj-info) "base_link"))))
