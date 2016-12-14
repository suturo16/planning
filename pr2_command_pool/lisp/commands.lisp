(in-package :pr2-command-pool-package)

(defun close-gripper (arm &optional  (strength 50))
  (action-move-gripper 0.0 arm strength))

(defun open-gripper (arm)
  (action-move-gripper 1.0 arm 1.0))

(defun is-object-in-view (object-id)
  T)

(defun get-object-location (object-name)
 (print object-name))

(defun get-object-dimensions (object-name)
  (print object-name))

(defun get-object-info (object-name)
  (make-object-info
   :name object-name))

(defun move-arm-to-object (obj-info arm)
  (let ((arm-str (if (string= arm +left-arm+) "left" "right")))
    (action-move-robot *move-robot-action-client*
                       (format nil "pr2_upper_body_~a_arm" arm-str)
                       (format nil "pr2_grasp_control_~a" arm)
                       (make-param +transform+ nil "object_frame"
                                   (format nil "~a ~a" (object-info-frame obj-info) "/base_link")) 
                       (make-param +double+ T "object_width" (object-info-width obj-info))
                       (make-param +double+ T "object_height" (object-info-height obj-info)))))

(defun get-drop-location (side)
  ; side in echten Namen übersetzen (left="red_dropzone", right="yellow_dropzone")
  (print side))

(defun move-object-with-arm (loc-info obj-info arm)
  (let ((arm-str (if (string= arm +left-arm+) "left" "right")))
    (action-move-robot *move-robot-action-client*
                       (format nil "pr2_upper_body_~a_arm" arm-str)
                       (format nil "pr2_place_control_~a" arm)
                       (make-param +transform+ nil "location_frame"
                                   (format nil "~a ~a" (object-info-frame loc-info) "/base_link"))
                       (make-param +transform+ nil "object_frame"
                                   (format nil "~a ~a" (object-info-frame obj-info) (format nil "/~a_wrist_roll_link" arm)))
                       (make-param +double+ T "object_width" (object-info-width obj-info))
                       (make-param +double+ T "object_height" (object-info-height obj-info))
                       (make-param +double+ T (format nil "~a_gripper_effort" arm) 50))))

(defun get-in-base-pose ()
  "Bring PR2 into base pose."
  (action-move-robot *move-robot-action-client* "pr2_upper_body" "pr2_base_pose_control"))
