(in-package :pepper-communication-package)

(defun execute-pepper-command (command)
  "Execute the given command."
  (plan-execution-package::execute command)
  ;command should be called "notify" but hasn't been updated yet.
  ;the IP of Pepper: "192.168.101.96"
  (fire-rpc "setStatus" "192.168.101.97" 8000))

