(in-package :planning-common-package)

; constants

; arms
(alexandria:define-constant +no-arm+ "n" :test #'equal)
(alexandria:define-constant +right-arm+ "r" :test #'equal)
(alexandria:define-constant +left-arm+ "l" :test #'equal)
(alexandria:define-constant +both-arms+ "b" :test #'equal)

; param types
(defconstant +double+ (symbol-code 'suturo_manipulation_msgs-msg:TypedParam :DOUBLE))
(defconstant +transform+ (symbol-code 'suturo_manipulation_msgs-msg:TypedParam :TRANSFORM))

(defparameter *transform-listener* nil)

(defun ensure-node-is-running ()
  "Ensure a node is running. Start one otherwise."
  (unless (eq (node-status) :RUNNING)
    (start-ros-node "planning")))

(defun make-param (type is-const name value)
  (make-message "suturo_manipulation_msgs/TypedParam"
                :type type
                :isConst is-const
                :name name
                :value value))

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
        

(defun file->string (path-to-file)
  (let ((in (open path-to-file :if-does-not-exist nil))
        (out ""))
    (when in
      (loop for line = (read-line in nil)
            while line do (setf out (concatenate 'string out line (string #\linefeed))))
      (close in))
    out))

(defun split (string &key (delimiterp #'delimiterp))
  (loop
    :for beg = (position-if-not delimiterp string)
    :then (position-if-not delimiterp string :start (1+ end))
    :for end = (and beg (position-if delimiterp string :start beg))
    :when beg :collect (subseq string beg end)
    :while end))

(defun delimiterp (c) (position c ":"))

(defun strings->KeyValues (strings)
  (when (>= (length strings) 2)
    (cons
     (make-message "diagnostic_msgs/KeyValue"
                   :key (car strings)
                   :value (car (cdr strings)))
     (let ((rest-strings (cdr (cdr strings))))
       (when rest-strings
         (strings->KeyValues rest-strings))))))

(defun get-joint-config (config-name)
  (let ((in (open (get-config-yaml-path config-name) :if-does-not-exist nil))
        (out nil))
    (when in
      (loop for line = (read-line in nil)
            while line do (when (not (string= "#" (subseq line 0 1)))
                              (setf out (cons (subseq line 2) out))))
      (close in))
    (reverse out)))

(defun get-controller-specs (controller-name)
  (file->string (get-controller-path controller-name)))

(defun get-controller-yaml-path (controller-name)
  (concatenate 'string
               (get-yaml-path "controller_specs")
               controller-name
               ".yaml"))

(defun get-config-yaml-path (config-name)
  (concatenate 'string
               (get-yaml-path "config")
               config-name
               ".yaml"))

(defun get-controller-path (controller-name)
  (let* ((file (concatenate 'string
                            (get-yaml-path "controller_specs")
                            controller-name))
         (yaml (probe-file (concatenate 'string file ".yaml")))
         (giskard (probe-file (concatenate 'string file ".giskard"))))
    (if yaml
        yaml
        giskard)))

(defun get-config-path (config-name)
  (let* ((file (concatenate 'string
                            (get-yaml-path "config")
                            config-name))
         (yaml (probe-file (concatenate 'string file ".yaml")))
         (giskard (probe-file (concatenate 'string file ".giskard"))))
    (if yaml
        yaml
        giskard)))
    
(defun get-yaml-path (type)
  (concatenate 'string
               (namestring (roslisp::ros-package-path "graspkard"))
               type
               "/"))
