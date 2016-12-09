(in-package :pr2-command-pool-package)

(defun close-gripper (arm &optional  (strength 1.0))
  "Let Raphael close the gripper with given id and maybe strength"
  (print arm)
  (print strength))

(defun open-gripper (arm)
  (print arm))

(defun is-object-in-view (object-id)
  T)

(defun get-object-location (object-name)
 (print object-name))

(defun get-object-dimensions (object-name)
  (print object-name))

(defun get-object-info (object-name)
  (print object-name))

(defun move-arm-to-object (obj-info arm)
  (let ((frame (object-info-frame obj-info))
        (width (object-info-width obj-info))
        (height (object-info-height obj-info))
        (depth (object-info-depth obj-info)))
  (action-move-robot *move-robot-action-client* "pr2_upper_body" "pr2_grasp_control" "object_frame" frame "width" width "depth" depth "height" height)))

(defun get-drop-location (side)
  ; side in echten Namen übersetzen (left="red_dropzone", right="yellow_dropzone")
  (print side))

(defun move-object-with-arm (loc-info obj-info arm)
  (action-move-robot *place-object-client* "pr2_upper_body" "pr2_place_control"
                     "object_frame" (object-info-frame obj-info)
                     "object_width" (object-info-width obj-info)
                     "object_height" (object-info-height obj-info)
                     "object_depth" (object-info-depth obj-info)
                     "location_frame" (object-info-frame loc-info)
                     "location_width" (object-info-width loc-info)
                     "location_height" (object-info-height loc-info)
                     "location_depth" (object-info-depth loc-info)))

(defun get-in-base-pose ()
  "brings the pr2 into base pose")
