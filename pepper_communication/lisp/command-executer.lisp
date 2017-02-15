(in-package :pepper-communication-package)

(defun execute-pepper-command (command)
  "Execute the given command."
  (plan-execution-package::execute command)

  (if (gethash :pepper *clients*)
      (fire-rpc-to-client "notify"
                          (client-host (gethash :pepper *clients*))
                          (client-port (gethash :pepper *clients*)))
      (fire-rpc "notify" "192.168.101.97" 8000)))

