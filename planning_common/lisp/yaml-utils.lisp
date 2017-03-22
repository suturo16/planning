(in-package :planning-common-package)

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
