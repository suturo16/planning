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

(defun handle-knowledge-update (json-string)
  "Adds neu guests to the guest list. Starts a new thread that updates the knowledgebase with given arguments.
GUEST-ID as string, name of the guest
REQUEST-ARGUMENTS as strings or integer, like ('setCake' 4) or ('getGuestInfo'). The first argument must be set to a request type
as defined here: https://docs.google.com/document/d/1wCUxW6c1LhdxML294Lvj3MJEqbX7I0oGpTdR5ZNIo_w"
  ;; (unless (member guest-id common:*guests* :test #'equal)
  ;;   (nconc common:*guests* '(guest-id)))
  (sb-thread:make-thread (lambda ()
                           (sb-thread:with-mutex (*prolog-mutex*)
                             (common:prolog-assert-dialog-element json-string))
                           (common:say "Thank you for the information.")
                           (when (gethash :pepper *clients*)
                             (fire-rpc-to-client :pepper "notify")))))
