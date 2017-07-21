(in-package :planning-common-package)

(defstruct object-info name frame type timestamp pose (height 0) (width 0) (depth 0) physical-parts add-info)

(defun get-object-part-detail (obj-info part detail)
  "Get the DETAIL of the (physical) PART of OBJ-INFO."
  ;; get the value
  (knowrob->str (second
                 ;; find the right detail
                 (find detail 
                       ;; find the right part
                       (find part (object-info-physical-parts obj-info)
                             :test (lambda (my-part part-list)
                                     (let ((name-of-obj
                                             (knowrob->str
                                              (second
                                               (find +name-of-object+ part-list :key #'first)))))
                                       (string-equal my-part (subseq name-of-obj 0 (1- (length name-of-obj)))))))
                       :key #'first))))
