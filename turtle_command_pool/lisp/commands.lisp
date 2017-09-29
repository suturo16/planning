(in-package :turtle-command-pool-package)

(defparameter *transform-listener* nil)

(defun get-transform-listener ()
  "Retrueve the transform listener."
  (unless *transform-listener*
    (setf *transform-listener* (make-instance 'cl-tf:transform-listener)))
  *transform-listener*)

(defun init-turtle ()
  (roslisp-utilities:startup-ros :name "tortugabot1/lisp_node" :anonymous nil))


(defun move (velocity &optional angle)
  "Let the turtle move with the given velocity and angle"
  (print velocity)
  (print angle))

(defun go-to-pr2 ()
  (call-move-base-action "map" (cl-transforms:make-3d-vector 1.186 2.229 0)(cl-transforms:make-quaternion 0 0 -0.708 0.707))
  (call-move-base-action "map" (cl-transforms:make-3d-vector 1.233 2.048 0)(cl-transforms:make-quaternion 0 0 -0.704 0.710)))

(defun go-to-table ()
  (call-move-base-action "map" (cl-transforms:make-3d-vector 0.072 -0.100 0)(cl-transforms:make-quaternion 0 0 -0.704 0.710)))


;; get pose of the object the turtle wants to go to
(defun get-pose-of (object)
  (let ((transform))
    (progn 
      (setq transform (cl-tf:lookup-transform (get-transform-listener) "map" object))
      (cl-transforms-stamped:make-pose-stamped "map" (roslisp:ros-time)
                                               (cl-tf:translation transform)
                                               (cl-tf:rotation transform)))))
