(in-package :pepper-communication-package)

(defun execute-pepper-command (command)
  "Execute the given command."
  (progn
    (with-fields (data)
        command 
      (plan-execution-package::execute data))
    
    (when (gethash :pepper *clients*)
      (fire-rpc-to-client :pepper "notify"))))
