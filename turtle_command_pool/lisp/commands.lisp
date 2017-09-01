(in-package :turtle-command-pool-package)

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
