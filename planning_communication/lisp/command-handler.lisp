(in-package :planning-communication-package)

(defun make-command-executer (cb)
  "Execute COMMAND with function CB."
  (lambda (command)
    (with-fields (data)
        command
      (funcall cb data))

    (when (gethash :pepper *clients*)
      (fire-rpc-to-client :pepper "notify"))))

(defun listen-for-commands (cb)
  "Return subscriber listening on topic /command with callback CB."
  (setf *command-subscriber* (subscribe "/command" "std_msgs/String" (make-command-executer cb))))
