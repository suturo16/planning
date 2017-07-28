(in-package :common)

(alexandria:define-constant +blade-of-cake-knife+ "BladeOfCakeKnife" :test #'string-equal)
(alexandria:define-constant +handle-of-cake-knife+ "HandleOfCakeKnife" :test #'string-equal)
(alexandria:define-constant +supporting-plane-of-cake-spatula+ "SupportingPlaneOfCakeSpatula" :test #'string-equal)
(alexandria:define-constant +handle-of-cake-spatula+ "HandleOfCakeSpatula" :test #'string-equal)


(alexandria:define-constant +name-of-object+ '|'nameOfObject'|)
(alexandria:define-constant +width-of-object+ '|'widthOfObject'|)
(alexandria:define-constant +height-of-object+ '|'heightOfObject'|)
(alexandria:define-constant +depth-of-object+ '|'depthOfObject'|)
(alexandria:define-constant +length-of-object+ '|'lengthOfObject'|)
(alexandria:define-constant +angle+ '|'angle'|)
(alexandria:define-constant +radius+ '|'radius'|)

(defun prolog-get-details (name)
  (cdr (car (car (prolog-get-info (format nil "[[nameOfObject,~a],radius,angle]" name))))))

(defun prolog-get-info-physical-parts (name)
  (prolog-get-info (format nil "[[nameOfObject,~a],physicalParts]" name)))

(defun get-phys-parts (name)
  (let ((parts (cdr (car (car (prolog-get-info-physical-parts name))))))
    (map 'list (lambda (x)
                 (cdr (car (car (prolog-get-info
                  (symbol-name (second x)))))))
         parts)))

(defun get-part-detail (obj-name part detail)
  (get-part-detail-inner (get-phys-parts obj-name) part detail))

(defun get-part-detail-inner (parts part detail)
  (let ((name (symbol-name (car (alexandria:assoc-value (car parts) +name-of-object+)))))
    (if (string-equal name part :start1 1 :end1 (- (length name) 2))
          (let ((value (symbol-name (car (alexandria:assoc-value (car parts) detail)))))
            (subseq value 1 (1- (length value))))
          (when (cdr parts)
              (get-part-detail-inner (cdr parts) part detail)))))
