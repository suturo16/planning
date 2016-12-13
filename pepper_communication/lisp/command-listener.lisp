(in-package :pepper-communication-package)

;execute-command-from-pepper
;we assume that "command" currently contains only the direction the object should be put down to. aka: right or left (ms1)
(defun listen-to-pepper ()
  "This is the service to listen to pepper."
;the comments are there due to a node starting bug with the action-server node
; (with-ros-node ("pepper_command_listener" :spin t)
    (subscribe "pepper_command" "std_msgs/String" #'execute-pepper-command))
;)

