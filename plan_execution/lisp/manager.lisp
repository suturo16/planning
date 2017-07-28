(in-package :pexecution)

(defparameter *current-guest-id* nil)
(defparameter *last-phase* :init)  ;; :init, :prep, :cut, :deliver
(defparameter *current-count* 0)

(defun next-guest-id ()
  "Set the current guest id to the lowest unserved guest."
  (setf *current-guest-id* (car (common:get-guest-ids))))

(defun get-order ()
  (common:get-guest-order *current-guest-id*))

(defun prep ()
  (execute "prep")
  (setf *last-phase* :prep))

(defun do-order ()
  (execute "cut")
  ;; increase order
  (setf *last-phase* :cut))

(defun finish-order ()
  (setf *current-guest-id* NIL)
  (setf *current-count* 0)
  (execute "deliver")
  (setf *last-phase* :deliver))

(defun start-caterros ()
  (loop
    unless *current-guest-id* do (next-guest-id)
      unless *current-guest-id* return nil
        when (< *current-count* (get-order)) do (do-order)
          when (>= *current-count* (get-order)) do (finish-order)))
