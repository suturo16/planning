(in-package :pepper-communication-package)

(defun listen-to-pepper ()
  "Starts listening on the topic /pepper_command, to recieve commands on.
The topic is usually filled by the rpc-server."
    (subscribe "/pepper_command" "std_msgs/String" #'execute-pepper-command))
