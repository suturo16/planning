(in-package :pr2-command-pool-package)

(defparameter *current-objects-in-world* NIL)

(defun topic-listen-to-objects-in-view ()
  "save seen objects in the param while listening on the given topic")
