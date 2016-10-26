(in-package :pepper-communication-package)

(defun listen-to-pepper ()
  "This is the service to listen to pepper."
  (with-ros-node ("pepper_command_listener" :spin t)
    (subscribe "pepper_command" "std_msgs/String" #'execute-pepper-command)))
