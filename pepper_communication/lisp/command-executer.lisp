(in-package :pepper-communication-package)

;execute-command-from-pepper
;we assume that "command" currently contains only the direction the object should be put down to. aka: right or left (ms1)
(defun execute-pepper-command (command)
  "Here the command retrieved from the pepper-listener should be executed."
  (print "this is the given command: " command)
  (pexecution::execute (list "grasp-object" "place-object" command))  
 )

