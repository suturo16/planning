(in-package :planning-common-package)

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

(defun run-full-pipeline ()
  (ros-info "run-full-pipeline" "recognizing Knife....")
  (service-run-pipeline "Knife")
  (sleep 10)
  (ros-info "run-full-pipeline" "recognizing Cake...")
  (service-run-pipeline "Cake")
  (sleep 10)
  (print "done recognizing things. You can start planning now!"))

(defun seen-since (obj-info)
  (let ((name (object-info-name obj-info))
        (frame-id (object-info-frame obj-info))
        (timestamp (object-info-timestamp obj-info)))
    (if (prolog-seen-since name frame-id timestamp)
        T
        NIL)))

(defun connect-objects (parent-info child-info)
  (service-connect-frames
   (format nil "/~a" (object-info-name parent-info))
   (format nil "/~a" (object-info-name child-info))))

(defun disconnect-objects (parent-info child-info)
  (prolog-disconnect-frames
   (format nil "/~a" (object-info-name parent-info))
   (format nil "/~a" (object-info-name child-info))))

(defun get-object-info (object-name)
  "Get object infos using prolog interface."
  (cut:with-vars-bound
      (?frame ?timestamp ?width ?height ?depth)
      (prolog-get-object-infos object-name)
    (make-object-info
       :name object-name
       :frame (string-downcase ?frame)
       :timestamp ?timestamp
       :height ?height
       :width ?width
       :depth ?depth)))
