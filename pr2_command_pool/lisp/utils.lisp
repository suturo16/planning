(in-package :pr2-command-pool-package)

; constants

; arms
(alexandria:define-constant +no-arm+ "n" :test #'equal)
(alexandria:define-constant +right-arm+ "r" :test #'equal)
(alexandria:define-constant +left-arm+ "l" :test #'equal)
(alexandria:define-constant +both-arms+ "b" :test #'equal)

; param types
(defconstant +double+ (symbol-code 'suturo_manipulation_msgs-msg:TypedParam :DOUBLE))
(defconstant +transform+ (symbol-code 'suturo_manipulation_msgs-msg:TypedParam :TRANSFORM))

(defun make-param (type is-const name value)
  (make-message "suturo_manipulation_msgs/TypedParam"
                :type type
                :isConst is-const
                :name name
                :value value))

(defun start-grip-pub (transform))

(defun stop-grip-pub ())

(defun calculate-transform (parent-frame frame)
  )

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
               config-name
               ".yaml"))

(defun get-yaml-path (type)
  (concatenate 'string
               (namestring (roslisp::ros-package-path "graspkard"))
               type
               "/"))
