(in-package :planning-communication-package)

(defun make-command-executer (cb)
  "Return function handling messages of type 'std_msgs/String' by calling CB with the message's data.

CB (function): Function capable of using message data to execute plans."
  (lambda (command)
    ;; Execute COMMAND with function CB.
    (with-fields (data)
        command
      (funcall cb data))

    (when (gethash :pepper *clients*)
      (fire-rpc-to-client :pepper "notify"))))

(defun listen-for-commands (cb)
  "Return subscriber listening on topic /command with callback CB.

CB (function): Look at `make-command-executer's docstring for further information."
  (setf *command-subscriber* (subscribe "/command" "std_msgs/String" (make-command-executer cb))))
