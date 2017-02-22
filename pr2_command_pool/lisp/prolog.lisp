(in-package :pr2-command-pool-package)

(alexandria:define-constant +knowrob-iri-prefix+ "http://knowrob.org/kb/knowrob.owl#" :test #'string=)

(defun prolog-get-values (prolog-function-name &rest arguments)
  "This is the function we mainly use. Call it with (prolog-get-values \"getObjectInfos\" \"zylinder\") and get the values
frame, height, width and depth as value binding."
  (cut:force-ll (cut:lazy-mapcar
                            (lambda (bindings)
                              (cut:with-vars-bound (common-lisp-user::?resp) bindings
                                common-lisp-user::?resp))
                            (json-prolog::prolog (apply 'append `((,prolog-function-name) ,arguments (common-lisp-user::?resp)))))))

; 'simple' because it uses the simple call
(defun prolog-get-object-infos-simple (name)
  (print (format nil
                 "prolog-get-object-infos-simple is now deprecated.~%
Use prolog-get-object-infos instead."))
  (cut:with-vars-bound (|?Frame| |?Timestamp| |?Width| |?Height| |?Depth|)
      (cut:lazy-car
       (json-prolog:prolog-simple 
        (format nil "get_object_infos(knowrob:~a, Frame, Timestamp, Width, Height, Depth)" name) :lispify T))
    (make-object-info
     :name name
     :frame (string-downcase |?Frame|)
     :timestamp |?Timestamp|
     :width |?Width|
     :height |?Height|
     :depth |?Depth|)))

(defun prolog-get-object-infos (name)
  "Call prolog function get_object_infos."
  (cut:lazy-car (json-prolog:prolog
                 `("get_object_infos"
                   ,(format nil "~a~a" +knowrob-iri-prefix+ name)
                   ?frame ?timestamp ?width ?height ?depth) :lispify T :package :pr2-do)))

(defun prolog-seen-since (name frame-id timestamp)
  "Call prolog function seen_since. Returns an empty list if successful or nil otherwise."
  (json-prolog:prolog
                 `("seen_since"
                  ,(format nil "~a~a" +knowrob-iri-prefix+ name)
                  ,frame-id ,timestamp) :lispify T :package :pr2-do))

(defun prolog-disconnect-frames (parent-frame-id child-frame-id)
  "Call prolog function disconnect_frames. Returns nothing."
  (json-prolog:prolog
                 `("disconnect_frames"
                   ,parent-frame-id ,child-frame-id) :lispify T :package :pr2-do))



;;                                         ; Some other templates. Try their worth.
;; (defun prolog-get-object-frame-eagerly (prolog-function-name type)
;;   "Returns the pose of the object with given id as PoseStamped. For now the function-name is getObjectPose"
;;   (cram-utilities:force-ll (json-prolog:prolog `(,prolog-function-name ,type ?pose))))

;;                                         ; Check responses and choose which style is better. Need Knowledge interface first.
;; (defun prolog-get-object-frame-lazy (prolog-function-name type)
;;   (cram-utilities:var-value 'common-lisp-user::?resp 
;;                             (cram-utilities:lazy-car
;;                              (json-prolog::prolog `(,prolog-function-name ,type common-lisp-user::?resp)))))

;;                                         ; Some template using the URI of our onthology. Adjust URI before usage!
;; (defparameter *url* "http://www.semanticweb.org/ontologies/insert/your/own/path/to/your/FancyOnthologyName.owl#")

;; (defun prolog-get-values-for-uri-eagerly (prolog-function-name argument)
;;   (let ((name (concatenate 'string *url* argument)))
;;     (cram-utilities:force-ll (cram-utilities:lazy-mapcar
;;                (lambda (bindings)
;;                  (cram-utilities:with-vars-bound (common-lisp-user::?resp) bindings
;;                    common-lisp-user::?resp))
;;                (json-prolog::prolog `(,prolog-function-name ,name common-lisp-user::?resp))))))


