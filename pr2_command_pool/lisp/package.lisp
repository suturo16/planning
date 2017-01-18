(defpackage :pr2-command-pool-package
  (:nicknames :pr2-do)
  (:use :cl :roslisp :cram-designators :cram-process-modules :cram-language-designator-support)
  (:import-from :cram-prolog :def-fact-group :<- :lisp-fun))
