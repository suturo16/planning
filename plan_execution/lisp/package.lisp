(defpackage :plan-execution-package
  (:nicknames :pexecution)
  (:use :cl :roslisp :cram-designators :cram-process-modules :cram-language-designator-support)
  (:import-from :cram-prolog :def-fact-group :<- :lisp-fun))
