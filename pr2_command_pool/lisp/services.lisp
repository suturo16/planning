(in-package :pr2-command-pool-package)

(defun service-get-object-pose (object-id)
  "call prolog service to receive the pose (including the location) of the object with given id")

(defun service-get-side-location (side)
  "call prolog service to get location of given side, left or right")
