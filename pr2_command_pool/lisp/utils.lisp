(in-package :pr2-command-pool-package)

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
            while line do (setf out (cons (subseq line 2) out)))
      (close in))
    (reverse out)))

(defun get-controller-specs (controller-name)
  (file->string (get-controller-yaml-path controller-name)))

(defun get-controller-yaml-path (controller-name)
  (concatenate 'string
               (get-yaml-path "controller_specs")
               controller-name))

(defun get-config-yaml-path (config-name)
  (concatenate 'string
               (get-yaml-path "config")
               config-name))

(defun get-yaml-path (type)
  (concatenate 'string
               (namestring (roslisp::ros-package-path "graspkard"))
               type
               "/"))
  
