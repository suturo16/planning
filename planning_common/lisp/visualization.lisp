(in-package :common)

(defparameter *plan-visualizer-pub* nil)

(defun get-plan-vis-pub ()
  (if *plan-visualizer-pub*
      *plan-visualizer-pub*
      (setf *plan-visualizer-pub* (advertise "/plan_visualization" "visualization_msgs/Marker"))))

(defstruct plan-state
  (scenario "CaterROS") plan step failures controller (feedback (make-msg "suturo_manipulation_msgs/MoveRobotFeedback")))

(defparameter *plan-state* (make-plan-state))

(defun set-plan-state (&key plan step)
  (when plan
    (setf (plan-state-plan *plan-state*) plan))
  (when step
    (setf (plan-state-step *plan-state*) step)))

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
  (visualize-state *plan-state*))
