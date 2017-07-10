(in-package :planning-communication-package)

(defun make-command-executer (cb)
  "Return function handling messages of type 'std_msgs/String' by calling CB with the message's data.

CB (function): Function capable of using message data to execute plans."
  (lambda (command)
    (with-fields (data) command
      ;; Let them know we got a command.
      (ros-info (command-executer) "Got new command: ~a" command)
      (common:say (format nil "I received a command: ~a." data))
      
      ;; Execute COMMAND with function CB.
      (funcall cb data))

    (when (gethash :pepper *clients*)
      (fire-rpc-to-client :pepper "notify"))))

(defun listen-for-commands (cb)
  "Return subscriber listening on topic /command with callback CB.

CB (function): Look at `make-command-executer's docstring for further information."
  (setf *command-subscriber* (subscribe "/command" "std_msgs/String" (make-command-executer cb))))

(defun handle-knowledge-update (guest-id request-type argument)
  "Checks the last thread and kills him, if he is stuck. Then starts a new thread that updates the knowledgebase with given arguments in a new thread.
GUEST-ID as string, name of the guest
REQUEST-TYPE as string, like 'setCake' or 'setDeposit'
ARGUMENT as variable datatype, amount of the cake, location name or nil."
  (when (and *knowledge-thread* (sb-thread:thread-alive-p *knowledge-thread*))
    (unless (sb-thread:join-thread *knowledge-thread* :timeout 3)
      (sb-thread:terminate-thread *knowledge-thread*)))
  (setf *knowledge-thread*
        (sb-thread:make-thread (lambda ()
                                 (common:prolog-update-guest-info guest-id request-type argument)
                                 (common:say "I am always at your service.")
                                 (sleep 2)
                                 (when (gethash :pepper *clients*)
                                   (fire-rpc-to-client :pepper "notify"))))))
