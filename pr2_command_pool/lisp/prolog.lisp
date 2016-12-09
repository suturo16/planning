(in-package :pr2-command-pool-package)

(defun prolog-get-values-eagerly (prolog-function-name &rest arguments)
  "This is the function we mainly use. Call it with (prolog-get-values \"getObjectInfos\" \"zylinder\") and get the values
frame, height, width and depth as value binding."
  (cram-utilities:force-ll (cram-utilities:lazy-mapcar
                            (lambda (bindings)
                              (cram-utilities:with-vars-bound (common-lisp-user::?resp) bindings
                                common-lisp-user::?resp))
                            (json-prolog::prolog (apply 'append `((,prolog-function-name) ,arguments (common-lisp-user::?resp)))))))

                                        ; Some other templates. Try their worth.
(defun prolog-get-object-frame-eagerly (prolog-function-name type)
  "Returns the pose of the object with given id as PoseStamped. For now the function-name is getObjectPose"
  (cram-utilities:force-ll (json-prolog:prolog `(,prolog-function-name ,type ?pose))))

                                        ; Check responses and choose which style is better. Need Knowledge interface first.
(defun prolog-get-object-frame-lazy (prolog-function-name type)
  (cram-utilities:var-value 'common-lisp-user::?resp 
                            (cram-utilities:lazy-car
                             (json-prolog::prolog `(,prolog-function-name ,type common-lisp-user::?resp)))))

                                        ; Some template using the URI of our onthology. Adjust URI before usage!
(defparameter *url* "http://www.semanticweb.org/ontologies/insert/your/own/path/to/your/FancyOnthologyName.owl#")

(defun prolog-get-values-for-uri-eagerly (prolog-function-name &rest arguments)
  (let ((name (concatenate 'string *url* arguments)))
    (cram-utilities:force-ll (cram-utilities:lazy-mapcar
               (lambda (bindings)
                 (cram-utilities:with-vars-bound (common-lisp-user::?resp) bindings
                   common-lisp-user::?resp))
               (json-prolog::prolog `(,prolog-function-name ,name common-lisp-user::?resp))))))


