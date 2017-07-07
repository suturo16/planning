(in-package :planning-common-package)

(defun get-joint-config (config-name)
  "Return joint configuration CONFIG-NAME as a list.

CONTROLLER-NAME (string): Should be one of the existent controller specifications. These can be found at URL
`suturo-docs.rtfd.io/en/latest/implementierung/schnittstellen.html#manipulation')."
  (let ((in (open (get-config-yaml-path config-name) :if-does-not-exist nil))
        (out nil))
    (when in
      (loop for line = (read-line in nil)
            while line do (when (not (string= "#" (subseq line 0 1)))
                              (setf out (cons (subseq line 2) out))))
      (close in))
    (reverse out)))

(defun get-controller-specs (controller-name)
  "Return controller specification CONTROLLER-NAME as a string.

CONFIG-NAME (string): Should be one of the existent joint configurations. These can be found at URL
`suturo-docs.rtfd.io/en/latest/implementierung/schnittstellen.html#manipulation')."
  (let ((path (get-controller-path controller-name)))
    (if path
        (file->string (get-controller-path controller-name))
        ;; else error
        (error 'giskard-config-error :controller controller-name))))

(defun get-controller-yaml-path (controller-name)
  "Return path to controller specification CONTROLLER-NAME as a string."
  (concatenate 'string
               (get-yaml-path "controller_specs")
               controller-name
               ".yaml"))

(defun get-config-yaml-path (config-name)
  "Return path to joint configuration CONFIG-NAME as a string."
  (concatenate 'string
               (get-yaml-path "config")
               config-name
               ".yaml"))

(defun get-controller-path (controller-name)
  "Return path to controller specification CONTROLLER-NAME, without '.yaml' ending, as a string.

For more information see "
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
  "Return path to directory TYPE in ros package 'graspkard' as a string.

TYPE (string): Should be either 'config' or 'controller_specs' but can be anything."
  (concatenate 'string
               (namestring (roslisp::ros-package-path "graspkard"))
               type
               "/"))
