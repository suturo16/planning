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

(define-condition perception-pipeline-failure (low-level-failure)
  ((description :initform "Pipeline could not be started."))
  (:documentation "Pipeline could not be started."))

(define-condition seen-since-failure (low-level-failure)
  ((description :initform "Seen since acting up."))
  (:documentation "Seen since acting up."))

(define-condition giskard-config-error (low-level-failure)
  ((description :initform "Giskard controller ~a not found.")
   (controller-name :initarg :controller
                    :reader giskard-config-error-controller))
  (:documentation "Giskard controller not found.")
  (:report (lambda (condition stream)
             (format stream (error-description condition) (giskard-config-error-controller condition)))))
