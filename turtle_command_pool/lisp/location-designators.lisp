(in-package :turtle-do)
;;dependencies: cram_location_costmap cram_tf cram_occupancy_grid_costmap
(defvar designator nil)
(defvar costmap nil)
(defvar solutions nil)
(defvar valid-solutions nil)
(defvar origin nil)


;;start a node in REPL with 
(defun init ()
  (roslisp-utilities:startup-ros :name "lisp_node" :anonymous nil))

;;start rviz and add Marker and MarkerArray, topic should contain lisp_node. define costmap parameters:
  (prolog:def-fact-group costmap-metadata ()
    (prolog:<- (location-costmap:costmap-size 15 15)) ; in meters
    (prolog:<- (location-costmap:costmap-origin -5 -10))
    (prolog:<- (location-costmap:costmap-resolution 0.05))
    (prolog:<- (location-costmap:costmap-padding 0.3))
    (prolog:<- (location-costmap:costmap-manipulation-padding 0.3))); padding to occupancy map obstacles
 
 
(defun see-used-costmap ()
   (setf costmap
               (cut:var-value 
                '?costmap
                (car (prolog:prolog (list 'location-costmap:desig-costmap designator '?costmap))))))


;;to visualize the costmap and see generated values:
(defun visualize-costmap ()
  (location-costmap:get-cost-map costmap))

;;to remove visualization call
(defun remove-visualization ()
  (location-costmap::remove-markers-up-to-index 10000))

;;create a cost function which returns for each {x, y} in location costmap grid a value between [0, 1]
(defun make-behind-cost-function-t (ref-x ref-y)
  "`ref-x' and `ref-y' are the coordinates of the reference point according to which the relation is resolved."
  (let* ((translated-supp-pose (cl-transforms:make-transform
                                (cl-transforms:make-3d-vector ref-x ref-y 0)
                                (cl-transforms:make-identity-rotation)))
         (world->supp-trans (cl-transforms:transform-inv translated-supp-pose)))
    (lambda (x y)
      (let* ((point (cl-transforms:transform-point world->supp-trans
                                                   (cl-transforms:make-3d-vector x y 0)))
             (vector-length (sqrt (+ (* (cl-transforms:x point) (cl-transforms:x point))
                                     (* (cl-transforms:y point) (cl-transforms:y point))))))
        (if (< (cl-transforms:x point) 0.0d0)
            (if (> (abs (/ (cl-transforms:x point) vector-length)) 0)
                (abs (/ (cl-transforms:x point) vector-length))
                0.0d0)
            0.0d0)))))

(defun make-left-of-cost-function-t (ref-x ref-y)
  "`ref-x' and `ref-y' are the coordinates of the reference point according to which the relation is resolved."
  (let* ((translated-supp-pose (cl-transforms:make-transform
                                (cl-transforms:make-3d-vector ref-x ref-y 0)
                                (cl-transforms:make-identity-rotation)))
         (world->supp-trans (cl-transforms:transform-inv translated-supp-pose)))
    (lambda (x y)
      (let* ((point (cl-transforms:transform-point world->supp-trans
                                                   (cl-transforms:make-3d-vector x y 0)))
             (vector-length (sqrt (+ (* (cl-transforms:x point) (cl-transforms:x point))
                                     (* (cl-transforms:y point) (cl-transforms:y point))))))
        (if (< (cl-transforms:x point) 0.0d0)
            (if (> (abs (/ (cl-transforms:x point) vector-length)) 0)
                (abs (/ (cl-transforms:x point) vector-length))
                0.0d0)
            0.0d0)))))

;; define order for your costmap function and give it a name:

(defun make-behind-cost-function ()
  (print "behind cost function"))
(defun make-left-of-cost-function ()
  (print "left of cost function"))

(defmethod location-costmap:costmap-generator-name->score ((name (common-lisp:eql 'behind-cost-function))) 10)
(defmethod location-costmap:costmap-generator-name->score ((name (common-lisp:eql 'left-of-cost-function))) 9)

(defun factgroup ()
  (prolog:<- (location-costmap:desig-costmap ?designator ?costmap)
    (desig:desig-prop ?designator (:go-to ?pose))
    (prolog:lisp-fun cl-transforms:origin ?pose ?pose-origin)
    (prolog:lisp-fun cl-transforms:x ?pose-origin ?ref-x)
    (prolog:lisp-fun cl-transforms:y ?pose-origin ?ref-y)
    (location-costmap:costmap ?costmap)
    (location-costmap:costmap-add-function
     behind-cost-function
     (make-behind-cost-function ?ref-x ?ref-y)
     ?costmap)))
;;define the prolog rule for generating costmaps:
;;(prolog:def-fact-group left-of-rules (location-costmap:desig-costmap)

;;using range costmap function:
;;range around a certain point
 (prolog:def-fact-group left-of-rules (location-costmap:desig-costmap)
    (prolog:<- (location-costmap:desig-costmap ?designator ?costmap)
      (desig:desig-prop ?designator (:go-to ?pose))
      (location-costmap:costmap ?costmap)
      (location-costmap:costmap-padding ?padding)
      (occupancy-grid-costmap::drivable-location-costmap ?costmap ?padding)
;      (location-costmap:padding ?padding)
      (and
       
       (location-costmap:costmap-add-function
        behind-cost-function
        (location-costmap:make-range-cost-function ?pose 1.0)
        ?costmap)
              
      (location-costmap:costmap-add-function 
        left-of-cost-function
        (location-costmap:make-range-cost-function ?pose 0.5 :invert t)
        ?costmap)
      ;(location-costmap:costmap-add-function
      ;left-of-cost-function
      ;(location-costmap:make-occupancy-grid-cost-function ?grid)
      ;?costmap)
       )))
    


;;resolve your new awesome designator:
(defun resolve-new-desig ()
     (desig:reference (desig:make-designator :location `((:go-to ,(cl-transforms:make-identity-pose))))))

;;using gaussian costmap function:
;;rainbow around a certain point
(defun test-group ()
  (prolog:def-fact-group right-of-rules (location-costmap:desig-costmap)
                        (prolog:<- (location-costmap:desig-costmap ?designator ?costmap)
                          (desig:desig-prop ?designator (:go-to ?pose))
                          (prolog:lisp-fun cl-transforms:origin ?pose ?pose-origin)
                          (location-costmap:costmap ?costmap)
                          (location-costmap:costmap-add-function
                           behind-cost-function
                           (location-costmap:make-gauss-cost-function ?pose-origin #2A((0.1 0) (0 0.1)))
                           ?costmap))))



;;opposite of the above
(defun test-group2 ()
  (prolog:def-fact-group negative-circle-left-of-rules (location-costmap:desig-costmap)
    (prolog:<- (location-costmap:desig-costmap ?designator ?costmap)
      (desig:desig-prop ?designator (:behind ?pose))
      (location-costmap:costmap ?costmap)
      (location-costmap:costmap-add-function
       behind-cost-function
       (location-costmap:make-range-cost-function ?pose 1.0 :invert t)
       ?costmap))))

 (defparameter test (desig:make-designator :location `((:go-to ,(cl-transforms:make-pose (cl-transforms:make-3d-vector -1.7 -6.8 0.0) (cl-transforms:make-quaternion 0.0 0.0 -0.035 0.9))))))

(defparameter test2 (desig:make-designator :location `((:go-to :table))))

(defun get-solutions (desig howmany)
   (let (newdesig)
     (dotimes (n howmany)
       (setf newdesig desig)
       (setf desig (next-solution newdesig))
       (push (reference newdesig) solutions))))

(defun reset-solutions ()
  (setf solutions nil))

(defun navigation-goal-validator-test (desig solution)
  (print "given desig:")
  (print desig)
  (print "found solution:")
  (print solution)
  (when (desig-prop-value desig :go-to)
    (if solution
        (if (and
             (> (cl-transforms:y (cl-transforms:origin solution))
                (+ (cl-transforms:y (cl-transforms:origin (desig-prop-value desig :go-to))) 0.2))
             (< (cl-transforms:y (cl-transforms:origin solution))
                (+ (cl-transforms:y (cl-transforms:origin (desig-prop-value desig :go-to))) 0.6))
             (< (cl-transforms:x (cl-transforms:origin solution))
                (+ (cl-transforms:x (cl-transforms:origin (desig-prop-value desig :go-to))) 0.2))
             (> (cl-transforms:x (cl-transforms:origin solution))
                (+ (cl-transforms:x (cl-transforms:origin (desig-prop-value desig :go-to))) 0.0)))
            (progn (print solution)
                   (push solution valid-solutions)
                   (print "accepted")
                   (sleep 2)
                   :accept)
            (progn
              (print "invalid pose")
              (print (cl-transforms:y (cl-transforms:origin solution)))
              :reject))
        (print "solution list is empty"))))

;;(register-location-validation-function
;;5 navigation-goal-validator)

;;6. create a designator and resolve it:
 (defun create-desig ()
   (setf designator (desig:make-designator :location `((:to :see)))))

;;7. to see which costmap was used, call this:
(defun resolve-desig ()
   (desig:reference designator))
;;CRAM costmaps need values from 0 to 1: 0 -- bad value, 1 -- perfect for designator description
