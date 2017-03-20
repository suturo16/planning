(in-package :planning-common-package)

(defparameter *transform-listener* nil)

(defun init-transform-listener ()
  (setf *transform-listener* (make-instance 'cl-tf:transform-listener)))

(defun get-transform-listener ()
  (if *transform-listener*
      *transform-listener*
      (init-transform-listener)))

(defun extract-pose-from-transform (parent-frame frame)
  (cl-tf:wait-for-transform (get-transform-listener)
                            :source-frame parent-frame
                            :target-frame frame
                            :timeout 1)
  (let ((target-transform-stamped
          (cl-tf:lookup-transform
           (get-transform-listener)
           parent-frame
           frame)))
     (cl-tf:transform->pose target-transform-stamped)))

(defun tf-pose->string (pose)
  (let ((origin (cl-tf:origin pose))
        (orientation (cl-tf:orientation pose)))
    (multiple-value-bind (axis angle) (cl-tf:quaternion->axis-angle orientation)
      (let* ((normalized-axis
               (if (eql angle 0.0d0)
                   (cl-tf:make-3d-vector 1 0 0)
                   (cl-tf:normalize-vector axis)))
             (normalized-angle (cl-tf:normalize-angle angle)))
        (format nil "~a ~a ~a ~a ~a ~a ~a"
                (cl-tf:x origin)
                (cl-tf:y origin)
                (cl-tf:z origin)
                (- 0 (cl-tf:x normalized-axis))
                (- 0 (cl-tf:y normalized-axis))
                (- 0 (cl-tf:z normalized-axis))
                (- 0 normalized-angle))))))

(defun tf-lookup->string (parent-frame frame)
  (tf-pose->string
   (extract-pose-from-transform parent-frame frame)))
