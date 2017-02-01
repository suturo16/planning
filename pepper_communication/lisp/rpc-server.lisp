(in-package :pepper-communication-package)

(defun init-rpc-server (&optional (restart-rosnode nil))
  "Starts and initializes the RPC server and rosnode, if needed or wanted."
  (when (or (eq (roslisp:node-status) :SHUTDOWN) restart-rosnode)
    (roslisp:start-ros-node "planning"))
  (import '(|sleepSomeTime|
            |cutCake|
            |stressLevel|
            |nextTask|
            |updateObserverClient|)
          :s-xml-rpc-exports))

(defun |sleepSomeTime| ()
  "Waits 3 seconds, before responding. For debugging pusposes."
  (sleep 3))
  
(defun |cutCake| ()
  "Publishes to the command listeners topic and responds with the amount of due tasks.
0 indicates immediate execution of given task."
  (let ((stress-level (length *commands-list*))
        (pub (advertise "/pepper_command" "std_msgs/String")))
    (publish-msg pub :data "cut-cake")
    stress-level))

(defun |stressLevel| ()
  "Returns the current stress level, represented by the length of tasks."
  (length *commands-list*))

(defun |nextTask| ()
  "Returns the identifier of the next task, as is in the commands list."
  (last *commands-list*))

(defun |updateObserverClient| (host port client-id)
  "Update clients' information about host and port, using the client id as primary key."
  (let ((client-key
          (case client-id
            ((0 "0" "pepper") :pepper)
            ((1 "1" "turtle") :turtle)
            (otherwise nil)))
        (error-message
          "ERROR:
Usage: updateConnection(host, port, client-key)
Valid values for client-key are:
0 or \"pepper\" for pepper
1 or \"turtle\" for the turtlebot"))
    (when (not client-key)
       (return-from |updateObserverClient| error-message))
    (when (stringp port)
      (setf port (parse-integer port)))
    (if (gethash client-key *clients*)
        ((lambda (client)
           (setf (client-host client) host)
           (setf (client-port client) port))
         (gethash client-key *clients*))
        (setf (gethash client-key *clients*)
              (make-client :host host :port port)))
    'SUCCESS))
