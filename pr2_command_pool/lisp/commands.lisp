(in-package :pr2-command-pool-package)

(defun close-gripper (arm &optional  (strength 1.0))
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
  (action-move-robot *move-robot-action-client* "pr2_upper_body" "pr2_grasp_control"
                     (make-param +transform+ nil "object_frame" (object-info-frame obj-info)) 
                     (make-param +double+ T "object_width" (object-info-width obj-info))
                     (make-param +double+ T "object_height" (object-info-height obj-info))
                     (make-param +double+ T "object_depth" (object-info-depth obj-info))))

(defun get-drop-location (side)
  ; side in echten Namen übersetzen (left="red_dropzone", right="yellow_dropzone")
  (print side))

(defun move-object-with-arm (loc-info obj-info arm)
  (action-move-robot *move-robot-action-client* "pr2_upper_body" "pr2_place_control"
                     (make-param +transform+ nil "object_frame" (object-info-frame obj-info))
                     (make-param +double+ T "object_width" (object-info-width obj-info))
                     (make-param +double+ T "object_height" (object-info-height obj-info))
                     (make-param +double+ T "object_depth" (object-info-depth obj-info))
                     (make-param +transform+ nil "location_frame" (object-info-frame loc-info))
                     (make-param +double+ T "location_width" (object-info-width loc-info))
                     (make-param +double+ T "location_height" (object-info-height loc-info))
                     (make-param +double+ T "location_depth" (object-info-depth loc-info))))

(defun get-in-base-pose ()
  "Bring PR2 into base pose."
  (action-move-robot *move-robot-action-client* "pr2_upper_body" "pr2_base_pose_control"))
