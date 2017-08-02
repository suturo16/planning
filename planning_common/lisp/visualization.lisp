(in-package :common)

(defparameter *plan-visualizer-pub* nil)

(defun get-plan-vis-pub ()
  (if *plan-visualizer-pub*
      *plan-visualizer-pub*
      (setf *plan-visualizer-pub* (advertise "/plan_visualization" "visualization_msgs/Marker"))))

(defstruct plan-state
  (scenario "CaterROS")
  plan
  step
  failures
  controller
  (feedback (make-msg "suturo_manipulation_msgs/MoveRobotFeedback")))

;; (defvar *plan-state* (cpl:make-fluent :name :plan-state :value (make-plan-state)))

;; (defparameter *plan-state* (cpl:make-fluent :name :plan-state :value (make-plan-state)))

(defvar *plan-state* (make-plan-state))

;; (defvar *plan-state-changed* (cpl:fl-or
;;                               (cpl:pulsed *plan-state*)
;;                               (cpl:pulsed (plan-state-scenario *plan-state*))
;;                               (cpl:pulsed (cpl:fl-funcall #'plan-state-step *plan-state*))
;;                               (cpl:pulsed (cpl:fl-funcall #'plan-state-failures *plan-state*))
;;                               (cpl:pulsed (cpl:fl-funcall #'plan-state-controller *plan-state*))
;;                               (cpl:pulsed (cpl:fl-funcall #'plan-state-feedback *plan-state*))))

(defun set-plan-state (&rest key-values)
  "Set multiple slots of *plan-state*. Pass arguments as `(slot ,value)."
  (when key-values
    (let ((slot-value (car key-values)))
      (set-plan-state-slot (first slot-value) (second slot-value))
      (apply #'set-plan-state (cdr key-values)))))

(defun set-plan-state-slot (slot value)
  "Set a single slot on *plan-state*."
  (setf (slot-value *plan-state* slot) value))

;;(cpl:whenever ((cpl:fl-or (cpl:pulsed test-fl2) (cpl:pulsed test-fl)))
;;  (print "test"))

;(cpl:whenever ((pulsed new-fluent)) (print "test"))

(defun plan-state->string (state)
  (with-fields
      (current_value alteration_rate)
      (plan-state-feedback state)
    (format NIL"[~a]
Plan: ~a; 
Step: ~a; ~a
Controller: ~a;
Feedback - EV: ~4d; AR: ~4d"
            (plan-state-scenario state)
            (plan-state-plan state)
            (plan-state-step state)
            (if (plan-state-failures state) (failures->string (plan-state-failures state))  NIL)
            (plan-state-controller state)
            current_value
            alteration_rate)))

(defun failures->string (failures)
  "Make a string containing the information in FAILURES.
e.g. 'retries: <retry-name> <fails>/<max-fails> --- <retry-name2> <fails>/<max-fails> ...'"
  (format nil "retries: ~{~A~^ --- ~}"
          (map 'list (lambda (x)
                       (format nil "~a ~a/~a used" (first x) (second x) (third x)))
               failures)))

(defun visualize (text)
  (publish (get-plan-vis-pub)
           (make-msg "visualization_msgs/Marker"
                     (type) 9  ;; Type: TEXT_VIEW_FACING
                     (color) (make-msg "std_msgs/ColorRGBA" :r 1 :g 1 :b 1 :a 1) ;; Color: white
                     (frame_id header) "base_link" ;; in relation to base_link
                     (z position pose) 1 ;; position in relation to frame_id
                     (id) 0 ;; unique id
                     (z scale) 0.8 ;; "fontsize"
                     (w orientation pose) 1 ;; make a legal quaternion
                     (text) text) ;; which text to show
           ))

(defun visualize-state (state)
  (visualize (plan-state->string state)))

(defun visualize-current-state ()
  (print (plan-state->string *plan-state*)))
  ;;(visualize-state *plan-state*))

