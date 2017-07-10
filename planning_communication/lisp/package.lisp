(defpackage :planning-communication-package
  (:nicknames :pcomm )
  (:use :cl :roslisp)
  (:export
   :setup-pepper-communication
   :fire-rpc
   :fire-rpc-to-client
   :*knowledge-thread*))

(in-package :planning-communication-package)

;; Active clients saved in a hash map. Update map and use those credentials for remote calls to pepper and turtle.
(defstruct client host port)
(defparameter *clients*  (alexandria:alist-hash-table '((:pepper . nil) (:turtle . nil))))

;; Client ids for mapping to keys in client hash map. See function |updateObserverClient|.
(alexandria:define-constant +pepper-client-id+ 0)
(alexandria:define-constant +pr2-client-id+ 1)
(alexandria:define-constant +turtle-client-id+ 2)

;; subscriber for the command topic
(defparameter *command-subscriber* nil)
(defparameter *knowledge-thread* nil)
