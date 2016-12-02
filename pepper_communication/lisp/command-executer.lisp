(in-package :pepper-communication-package)

;execute-command-from-pepper
;we assume that "command" currently contains only the direction the object should be put down to. aka: right or left (ms1)
(defun execute-pepper-command (command)
  "Here the command retrieved from the pepper-listener should be executed."
  (list "pick-up-object" "put-down-object" command)
  (plan-execution-package::execute command)  
  (print command))

