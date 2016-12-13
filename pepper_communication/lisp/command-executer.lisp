(in-package :pepper-communication-package)

(defvar *commands-list* NIL)

;execute-command-from-pepper
;we assume that "command" currently contains only the direction the object should be put down to. aka: right or left (ms1)
(defun execute-pepper-command (command)
 "Here the command retrieved from the pepper-listener should be executed."
  (with-fields ((command-string (data))) command
    (print command-string)
    (if (eq (last *commands-list*) NIL)
        (push (list "grasp-object" "place-object" command-string) *commands-list*)
        (progn
          (handler-case (pexecution::execute (first (last *commands-list*)))
           (plan-execution-package::toplvl-being-executed ()
             (progn
               (print "toplvl is busy. Command is saved and will be executed later")
               (push (list "grasp-object" "place-object" command-string) *commands-list*)))
           (plan-execution-package::toplvl-completed-execution ()
             (progn
               (print "toplvl has completed execution and can be called with new command.")
               (setq *commands-list* (butlast *commands-list*)))))))))

