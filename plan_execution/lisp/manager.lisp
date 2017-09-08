(in-package :pexecution)

(defparameter *current-guest-id* nil)
(defparameter *last-phase* :init)  ;; :init, :prep, :cut, :deliver
(defparameter *current-count* 0)

(defun next-guest-id ()
  "Set *current-guest-id* to the next unserved guest."
  (loop
    while (not *current-guest-id*)
    do (setf *current-guest-id* (common:get-current-order))
    unless *current-guest-id* do (progn (print "Waiting for guest...") (sleep 5))))

(defun get-order ()
  (common:get-guest-order *current-guest-id*))

(defun prep ()
  (execute "prep")
  (setf *last-phase* :prep))

(defun do-order ()
  (execute "cut")
  (common:prolog-increase-delivered-amount 1)
  (setf *last-phase* :cut))

(defun finish-order ()
  (execute "deliver")
  (notify-of-completion)
  (setf *current-guest-id* NIL)
  (setf *last-phase* :deliver))

(defun use-generator ()
  "Use the plan generator to generate a plan for serving the desired amount of cake slices."
  (ros-info (plan-manager) "Using plan generator to plan to serve cake.")
  (with-pr2-process-modules
    (process-module-alias :manipulation 'giskard-manipulation)
    (execute-desigs
     (transform-plan-to-action-designators
      (pgeneration::generate-plan-for-cake-serving
       (common:get-remaining-amount-for-order *current-guest-id*)))))
  (notify-of-completion))

(defun notify-of-completion ()
  (pcomm:fire-rpc-to-client :pepper "new_notify"
                            (format NIL
                                    "{guestId:'~a',return:{type:'complete',success:'1'}}"
                                    *current-guest-id*)))

(defun start-caterros-wip (&optional (use-generator NIL))
  (loop
    unless *current-guest-id*
      do (next-guest-id)
    end
    if use-generator
      do (use-generator) 
    else
      unless (member *last-phase* '(:prep :cut))
        do (prep)
        and
          when (not (= (common:get-remaining-amount-for-order *current-guest-id*) 0))
            do (do-order)
            and
              when (<= (common:get-remaining-amount-for-order *current-guest-id*) 0)
                do (finish-order)))

(defun start-caterros (&optional (use-generator NIL))
   (if use-generator
       (top-level (progn
                    (when (not *current-guest-id*)
                      (next-guest-id))
                    (use-generator)))
         (loop
           unless *current-guest-id*
             do (next-guest-id)
           unless (member *last-phase* '(:prep :cut))
             do (prep)
           when (not (= (common:get-remaining-amount-for-order *current-guest-id*) 0))
             do (do-order)
           when (<= (common:get-remaining-amount-for-order *current-guest-id*) 0)
             do (finish-order))))

(defun test-guest ()
  (if (common::prolog-get-customer-infos 1)
      (common::prolog-set-delivered-amount 1 0)
      (common::prolog-assert-dialog-element "{guestId:\"1\",query:{type:\"setCake\",amount:2,guestName:\"arthur\"}}")))

(defun transform-plan-to-action-designators (plan)
  "Transform PLAN to list of action designators."
  (map 'cons (lambda (task) (make-designator :action
                                             (map 'cons
                                                  (lambda (x) (list
                                                               (intern (string-upcase (first x)) :keyword)
                                                               (if (string-equal (first x) "type")
                                                                   (intern (string-upcase (cdr x)) :keyword)
                                                                   (cdr x))))
                                                  task))) (yason:parse plan :object-as :alist)))
