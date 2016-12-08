(in-package :pr2-command-pool-package)

(defun prolog-get-object-pose (type)
  "Returns the pose of the object with given id as PoseStamped."
  (cram-utilities:force-ll (json-prolog::prolog `(getObjectPose ,type ?pose))))

