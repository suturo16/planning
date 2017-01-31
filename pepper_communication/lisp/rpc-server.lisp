(in-package :pepper-communication-package)

(defun init-rpc-server (&optional (restart-rosnode nil))
  "Starts and initializes the RPC server and rosnode, if needed or wanted."
  (when (or (eq (roslisp:node-status) :SHUTDOWN) restart-rosnode)
    (roslisp:start-ros-node "planning"))
  (import '(|sleepSomeTime|
            |cutCake|
            |updateConnection|)
          :s-xml-rpc-exports))

(defun |sleepSomeTime| ()
  (sleep 3))
  
(defun |cutCake| ()
  "Publishes to the command listeners topic and responds with the amount of due tasks.
0 indicates immediate execution of given task."
  (let ((stress-level (length *commands-list*))
        (pub (advertise "/pepper_command" "std_msgs/String")))
    (publish-msg pub :data "cut-cake")
    stress-level))

(defun |updateConnection| (host port client-id)
  "Update clients' information about host and port, using the client id as primary key."
  (if (member client-id (alexandria:hash-table-keys *clients*))
      "update client"
      "add client"))
