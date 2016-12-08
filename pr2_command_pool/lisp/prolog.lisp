(in-package :pr2-command-pool-package)

(defun prolog-get-object-pose (type)
  "Returns the pose of the object with given id as PoseStamped."
  (cram-utilities:force-ll (json-prolog:prolog `("getObjectPose" ,type ?pose))))

                                        ; Check responses and choose which style is better. Need Knowledge interface first.
(defun prolog-get-object-pose-fine (type)
  (cram-utilities:var-value 'common-lisp-user::?resp 
                            (cram-utilities:lazy-car
                             (json-prolog::prolog `("getObjectPose" ,type common-lisp-user::?resp)))))

                                        ; Some template using the URI of our onthology. Adjust URI before usage!
(defparameter *url* "http://www.semanticweb.org/ontologies/insert/your/own/path/to/your/FancyOnthologyName.owl#")

(defun prolog-get-values-eagerly (prolog-function-name some-input)
  (let ((name (concatenate 'string *url* some-input)))
    (cram-utilities:force-ll (cram-utilities:lazy-mapcar
               (lambda (bindings)
                 (cram-utilities:with-vars-bound ( common-lisp-user::?resp ) bindings
                   common-lisp-user::?resp))
               (json-prolog::prolog `(,prolog-function-name ,name common-lisp-user::?resp))))))

