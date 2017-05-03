(in-package :common)

(define-condition low-level-failure (cpl:simple-plan-failure)
  ((description :initarg :description
                :initform "Low-level function did not succeed."
                :reader error-description))
  (:documentation "Low-level function did not succeed.")
  (:report (lambda (condition stream)
             (format stream (error-description condition)))))

(define-condition high-level-failure (cpl:simple-plan-failure)
  ((description :initarg :description
                :initform "High-level plan did not succeed."
                :reader error-description))
  (:documentation "High-level plan did not succeed.")
  (:report (lambda (condition stream)
             (format stream (error-description condition)))))

(define-condition action-timeout (low-level-failure)
  ((description :initform "Action call timed out."))
  (:documentation "Action call timed out."))

(define-condition action-lost (low-level-failure)
  ((description :initform "Action goal was lost."))
  (:documentation "Action goal was lost."))

