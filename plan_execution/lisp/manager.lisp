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
  ;; (notify-order-ready *current-guest-id*)
  (setf *current-guest-id* NIL)
  (setf *last-phase* :deliver))

(defun start-caterros ()
  (loop
    unless *current-guest-id* do (next-guest-id)
      unless (member *last-phase* '(:prep :cut)) do (prep)
        when (not (= (common:get-remaining-amount-for-order *current-guest-id*) 0)) do (do-order)
          when (<= (common:get-remaining-amount-for-order *current-guest-id*) 0) do (finish-order)))

(defun test-guest ()
  (if (common::prolog-get-customer-infos 1)
      (common::prolog-set-delivered-amount 1 0)
      (common::prolog-assert-dialog-element "{guestId:\"1\",query:{type:\"setCake\",amount:2,guestName:\"arthur\"}}")))

(defun transform-plan-to-action-designators (plan)
"Transform PLAN to list of action designators."
 (map 'cons (lambda (task) (make-designator :action
               (map 'cons
                    (lambda (x) (cons (intern (string-upcase (first x))) (cdr x)))
                    task))) (yason:parse plan :object-as :alist)))
