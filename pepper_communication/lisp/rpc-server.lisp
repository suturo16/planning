(in-package :pepper-communication-package)

;; Active clients saved in a hash map. Update map and use those credentials for remote calls to pepper ans turtle.
(defstruct client host port)
(defparameter *clients*  (alexandria:alist-hash-table '((:pepper . nil) (:turtle . nil))))

;; Client ids for mapping to keys in client hash map. See function |updateObserverClient|.
(alexandria:define-constant +pepper-client-id+ 0)
(alexandria:define-constant +pr2-client-id+ 1)
(alexandria:define-constant +turtle-client-id+ 2)

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
  (let ((pub (advertise "/pepper_command" "std_msgs/String")))
    (if  (not (eq (roslisp:node-status) :SHUTDOWN))
         (progn (publish-msg pub :data "cut-cake")
                (|stressLevel|))
         -1)))

(defun |stressLevel| ()
  "Returns the current stress level, represented by the length of tasks in task-buffer."
  (roslisp-queue:queue-size (roslisp::buffer (roslisp::subscriber-subscription *pepper-subscriber*))))

(defun |nextTask| ()
  "Returns the identifier of the next task, as is in the commands list."
  "Not implemented!")

(defun |updateObserverClient| (client-id host port)
  "Update clients' information about host and port, using the client id as primary key."
  (let ((client-key
          (case client-id
            ((+pepper-client-id+ "0" "pepper") :pepper)
            ((+turtle-client-id+ "2" "turtle") :turtle)
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
