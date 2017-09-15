(defpackage :planning-common-package
  (:nicknames :common)
  (:use :cl :roslisp)
  (:export

   :+no-arm+
   :+left-arm+
   :+right-arm+
   :+both-arms+

   :+double+
   :+transform+

   :*guests*

   :low-level-failure
   :high-level-failure
   :action-timeout
   :action-lost
   :perception-pipeline-failure
   :seen-since-failure
   
   :object-info
   :make-object-info
   :object-info-name
   :object-info-frame
   :object-info-timestamp
   :object-info-height
   :object-info-width
   :object-info-depth
   :object-info-type
   :object-info-pose
   :get-object-detail
   :get-object-part-detail

   :service-run-pipeline

   :setup-move-robot-client
   :action-move-robot

   :prolog-seen-since
   :prolog-disconnect-frames
   :prolog-connect-frames
   :prolog-assert-dialog-element
   :prolog-get-customer-infos
   :prolog-get-open-orders-of
   :prolog-get-free-table
   :prolog-set-delivered-amount
   :prolog-increase-delivered-amount
   :prolog-delete-object
   
   :ensure-node-is-running
   :make-param
   :get-transform-listener
   :extract-pose-from-transform
   :tf-pose->string
   :tf-lookup->string
   :file->string
   :split
   :strings->KeyValues
   :get-joint-config
   :get-controller-specs
   :say
   :get-guest-order
   :get-current-order
   :get-place-of-guest
   
   :+blade-of-cake-knife+
   :+handle-of-cake-knife+
   :+supporting-plane-of-cake-spatula+
   :+handle-of-cake-spatula+
   :+name-of-object+
   :+width-of-object+
   :+height-of-object+
   :+depth-of-object+
   :+length-of-object+
   :+angle+
   :+radius+
   :get-phys-parts
   
   :run-full-pipeline
   :run-pipeline
   :connect-objects
   :disconnect-objects
   :get-object-info
   :get-current-order
   :get-free-table
   :get-remaining-amount-for-order))

(in-package :planning-common-package)

; constants
; arms
(alexandria:define-constant +no-arm+ "n" :test #'string-equal)
(alexandria:define-constant +right-arm+ "r" :test #'string-equal)
(alexandria:define-constant +left-arm+ "l" :test #'string-equal)
(alexandria:define-constant +both-arms+ "b" :test #'string-equal)

; param types
(defconstant +double+ (symbol-code 'suturo_manipulation_msgs-msg:TypedParam :DOUBLE))
(defconstant +transform+ (symbol-code 'suturo_manipulation_msgs-msg:TypedParam :TRANSFORM))

;; The guest name queue
(defparameter *guests* (list) "Names of all guests, used throughout the scene.")
