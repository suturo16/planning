(defpackage :plan-execution-package
  (:nicknames :pexecution)
  (:use :cpl :roslisp :cram-designators :cram-process-modules :cram-language-designator-support)
  (:import-from :cram-prolog :def-fact-group :<- :lisp-fun)
  (:import-from :turtle-do :simple-navigation)
  (:export giskard-manipulation
   :execute))

