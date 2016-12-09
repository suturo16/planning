(in-package :pr2-command-pool-package)

(defun close-gripper (arm &optional  (strength 1.0))
  "Let Raphael close the gripper with given id and maybe strength"
  (print arm)
  (print strength))

(defun open-gripper (arm)
  (print arm))

(defun is-object-in-view (object-id)
  (print object-id))

(defun get-object-info (object-id)
  (let ((pl-resp (prolog-get-values "getObjectInfos" object-id)))
    (make-object-info
       :name object-id
       :frame (first pl-resp)
       :height (second pl-resp)
       :width (third pl-resp)
       :depth (fourth pl-resp))))

(defun grasp-object (arm object-location)
  (print arm)
  (print object-location))

(defun lift-object (arm &optional  (height 1.0))
  (print arm)
  (print height))

(defun get-drop-location (side)
  (get-object-info side))

(defun put-object-down-to (arm location)
  (print arm)
  (print location))

(defun get-in-base-pose ()
  "brings the pr2 into base pose")
