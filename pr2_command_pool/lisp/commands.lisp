(in-package :pr2-command-pool-package)

(defun close-gripper (arm &optional  (strength 50))
  "Call action to close gripper."
  (action-move-gripper 0.0 arm strength))

(defun open-gripper (arm)
  "Call action to open gripper."
  (action-move-gripper 0.09 arm 70))

(defun run-full-pipeline ()
  (service-run-pipeline))

(defun check-object-location (object-info)
  (when object-info
    (get-in-base-pose)
    ;turn head
    (service-run-pipeline "knife" "box")
    (when (seen-since object-info)
      T)))

(defun seen-since (obj-info)
  (let ((name (object-info-name obj-info))
        (frame-id (object-info-frame obj-info))
        (timestamp (object-info-timestamp obj-info)))
    (if (prolog-seen-since name frame-id timestamp)
        T
        NIL)))

(defun connect-objects (parent-info child-info)
  (service-connect-frames
   (format nil "/~a" (object-info-name parent-info))
   (format nil "/~a" (object-info-name child-info))))

(defun disconnect-objects (parent-info child-info)
  (prolog-disconnect-frames
   (format nil "/~a" (object-info-name parent-info))
   (format nil "/~a" (object-info-name child-info))))

(defun connect-obj-with-gripper (obj-info arm)
  (service-connect-frames
   (format nil "/~a_wrist_roll_link" arm)
   (format nil "/~a" (object-info-name obj-info))))
   
(defun get-object-info (object-name)
  "Get object infos using prolog interface."
  (cut:with-vars-bound
      (?frame ?timestamp ?width ?height ?depth)
      (prolog-get-object-infos object-name)
    (make-object-info
       :name object-name
       :frame (string-downcase ?frame)
       :timestamp ?timestamp
       :height ?height
       :width ?width
       :depth ?depth)))

(defun move-arm-to-object (obj-info arm)
  "Call action to move arm to an object."
  (let ((arm-str (if (string= arm +left-arm+) "left" "right")))
    (action-move-robot (format nil "pr2_upper_body_~a_arm" arm-str)
                       (format nil "pr2_grasp_control_~a" arm)
                       (make-param +transform+ nil "object_frame"
                                   (format nil "~a ~a" (object-info-name obj-info) "base_link")) 
                       (make-param +double+ T "object_width" (write-to-string (object-info-width obj-info)))
                       (make-param +double+ T "object_height" (write-to-string (object-info-height obj-info))))))

(defun move-object-with-arm (loc-info obj-info arm)
  "Call action to place an object at a location."
  (let ((arm-str (if (string= arm +left-arm+) "left" "right")))
    (action-move-robot (format nil "pr2_upper_body_~a_arm" arm-str)
                       (format nil "pr2_place_control_~a" arm)
                       (make-param +transform+ T "location_frame"
                                   (format nil "~a ~a" (object-info-name loc-info) "base_link"))
                       (make-param +transform+ T "object_frame"
                                   (format nil "~a ~a" (object-info-name obj-info) (format nil "~a_wrist_roll_link" arm)))
                       (make-param +double+ T "object_width" (write-to-string (object-info-width obj-info)))
                       (make-param +double+ T "object_height" (write-to-string (object-info-height obj-info)))
                       (make-param +double+ T (format nil "~a_gripper_effort" arm) (write-to-string 50)))))

(defun get-in-base-pose ()
  "Bring PR2 into base (mantis) pose."
  (action-move-robot "pr2_upper_body" "pr2_upper_body_joint_control"
                     (make-param +double+ T "torso_lift_joint" "0.25")
                     (make-param +double+ T "r_shoulder_pan_joint" "-1.23679")
                     (make-param +double+ T "r_shoulder_lift_joint" "-0.247593")
                     (make-param +double+ T "r_upper_arm_roll_joint" "-0.614271")
                     (make-param +double+ T "r_elbow_flex_joint" "-1.38094")
                     (make-param +double+ T "r_forearm_roll_joint" "4.94757")
                     (make-param +double+ T "r_wrist_flex_joint" "-1.56861")
                     (make-param +double+ T "r_wrist_roll_joint" "0")
                     (make-param +double+ T "l_shoulder_pan_joint" "1.23679")
                     (make-param +double+ T "l_shoulder_lift_joint" "-0.247593")
                     (make-param +double+ T "l_upper_arm_roll_joint" "0.614271")
                     (make-param +double+ T "l_elbow_flex_joint" "-1.38094")
                     (make-param +double+ T "l_forearm_roll_joint" "-4.94757")
                     (make-param +double+ T "l_wrist_flex_joint" "-1.56861")
                     (make-param +double+ T "l_wrist_roll_joint" "0")))

(alexandria:define-constant +blade-%+ 0.65)

(defun grasp-knife (knife-info arm)
  "Grasp a knife with the right arm."
  (let* ((arm-str (if (string= arm +left-arm+) "left" "right"))
         (grip-length (* (1- +blade-%+) (object-info-width knife-info)))
         (blade-height (object-info-height knife-info)))
    (action-move-robot "knife_grasp"
                       (format nil "pr2_upper_body_~a_arm" arm-str)
                       (make-param +transform+ NIL "knife_frame" (format nil  "~a ~a" (object-info-name knife-info) "base_link"))
                       (make-param +double+ T "blade_height" (write-to-string blade-height))
                       (make-param +double+ T "grip_length" (write-to-string grip-length)))))

(defun detach-knife-from-rack (knife-info arm)
  "Move the knife away from the rack."
  (let* ((arm-str (if (string= arm +left-arm+) "left" "right")))
    (action-move-robot (format nil "pr2_knife_detatch_~a" arm)
                       (format nil "pr2_upper_body_~a_arm" arm-str)
                       (make-param +transform+ NIL "knife_frame" (format nil  "~a ~a" (object-info-name knife-info)
                                                                         (format nil "~a_wrist_roll_link" arm)))
                       (make-param +transform+ T "original_knife_tf" (tf-lookup->string "base_link" (object-info-name knife-info))))))

(defun take-cutting-position (cake-info knife-info arm slice-width)
  "Take a position above the cake, ready to cut it."
  (let ((arm-str (if (string= arm +left-arm+) "left" "right"))
        (handle-length (* (1- +blade-%+) (object-info-width knife-info))))
    (action-move-robot (format nil "pr2_cut_position_~a" arm)
                       (format nil "pr2_upper_body_~a_arm" arm-str)
                       (make-param +transform+ NIL "cake_tf" (format nil "~a ~a" (object-info-name cake-info) "base_link"))
                       (make-param +double+ T "cake_length_x" (write-to-string (object-info-width cake-info)))
                       (make-param +double+ T "cake_width_y" (write-to-string (object-info-depth cake-info)))
                       (make-param +double+ T "cake_height_z" (write-to-string (object-info-height cake-info)))
                       (make-param +transform+ NIL "knife_tf" (format nil "~a ~a" (object-info-name knife-info) (format nil "~a_wrist_roll_link" arm)))
                       (make-param +double+ T "knife_height" (write-to-string (object-info-height knife-info)))
                       (make-param +double+ T "handle_length" (write-to-string handle-length))
                       (make-param +double+ T "slice_width" (write-to-string slice-width)))))

(defun cut-cake (cake-info knife-info arm slice-width)
  "Cut the cake."
  (let ((arm-str (if (string= arm +left-arm+) "left" "right"))
        (handle-length (* (1- +blade-%+) (object-info-width knife-info))))
    (action-move-robot (format nil "pr2_cut_~a" arm)
                       (format nil "pr2_upper_body_~a_arm" arm-str)
                       (make-param +transform+ NIL "cake_tf" (format nil "~a ~a" (object-info-name cake-info) "base_link"))
                       (make-param +double+ T "cake_length_x" (write-to-string (object-info-width cake-info)))
                       (make-param +double+ T "cake_width_y" (write-to-string (object-info-depth cake-info)))
                       (make-param +double+ T "cake_height_z" (write-to-string (object-info-height cake-info)))
                       (make-param +transform+ NIL "knife_tf" (format nil "~a ~a" (object-info-name knife-info) (format nil "~a_wrist_roll_link" arm)))
                       (make-param +double+ T "knife_height" (write-to-string (object-info-height knife-info)))
                       (make-param +double+ T "handle_length" (write-to-string handle-length))
                       (make-param +double+ T "slice_width" (write-to-string slice-width)))))

; Won't be implemented for now.
;(defun push-aside (cake-info cake-piece-info arm)
;  (let ((arm-str (if (string= arm +left-arm+) "left" "right")))
;    (action-move-robot (format nil "pr2_shove_~a" arm)
;                       (format nil "pr2_upper_body_~a_arm" arm-str)
;                       (make-param +transform+ NIL "cake_tf" (format nil "~a ~a" (object-info-name cake-info) "base_link"))
;                       (make-param +double+ T "cake_length" (object-info-depth cake-info))
;                       (make-param +double+ T "cake_width" (object-info-width cake-info))
;                       (make-param +double+ T "cake_height" (object-info-height cake-info))
;                       (make-param +transform+ NIL "cake_piece_tf" (format nil "~a ~a" (object-info-name cake-piece-info) "base_link"))
;                       (make-param +double+ T "cake_piece_length" (object-info-depth cake-piece-info))
;                       (make-param +double+ T "cake_piece_width" (object-info-width cake-piece-info))
;                       (make-param +double+ T "cake_piece_height" (object-info-height cake-piece-info)))))
