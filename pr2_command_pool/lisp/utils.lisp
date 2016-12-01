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

(defun get-joints (config-name)
  (let ((in (open (get-config-yaml-path config-name) :if-does-not-exist nil))
        (out nil))
    (when in
      (loop for line = (read-line in nil)
            while line do (setf out (cons (subseq line 2) out)))
      (close in))
    (reverse out)))

(defun get-controller-yaml-path (controller-name)
  (concatenate 'string
               (get-yaml-path "controller_specs")
               controller-name))

(defun get-config-yaml-path (config-name)
  (concatenate 'string
               (get-yaml-path "config")
               config-name))

; TODO(cpo): Make more stable, only works for systems on root level in planning repo
(defun get-yaml-path (type)
  (let ((path (sb-unix:posix-getcwd)))
    (concatenate 'string
                 (subseq path
                         0
                         (position "/" path :from-end t :test #'string-equal))
                 "/yaml/"
                 type
                 "/")))
  
