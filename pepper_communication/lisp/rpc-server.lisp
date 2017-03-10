(in-package :pepper-communication-package)

;; Active clients saved in a hash map. Update map and use those credentials for remote calls to pepper and turtle.
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
            |do|
            |cutCake|
            |stressLevel|
            |nextTask|
            |updateObserverClient|)
          :s-xml-rpc-exports))

(defun |sleepSomeTime| (dummy)
  "Waits 3 seconds, before responding. For debugging pusposes."
  (sleep 3))

(defun |cutCake| (dummy)
  (|do| "cut cake"))


(defun |do| (command)
  "Publishes to the command listeners topic and responds with the amount of due tasks.
0 indicates immediate execution of given task."
  (let ((pub (advertise "/pepper_command" "std_msgs/String")))
    (if  (not (eq (roslisp:node-status) :SHUTDOWN))
         (progn (publish-msg pub :data command)
                (|stressLevel| "asdf"))
         -1)))

(defun |stressLevel| (dummy)
  "Returns the current stress level, represented by the length of tasks in task-buffer,
or -1 if the subscriber is unavailable."
  (when (or
         (not *pepper-subscriber*)
         (not (roslisp::thread-alive-p (roslisp::topic-thread (roslisp::subscriber-subscription *pepper-subscriber*)))))
    (return-from |stressLevel| -1))
  
  (roslisp-queue:queue-size (roslisp::buffer (roslisp::subscriber-subscription *pepper-subscriber*))))

(defun |nextTask| ()
  "Returns the identifier of the next task, as is in the commands list."
  "Not implemented!")

(defun |updateObserverClient| (client-id host port)
  "Update clients' information about host and port, using the client id as primary key."
  (let ((client-key
          (alexandria:switch (client-id :test #'equal)
            (+pepper-client-id+ :pepper)
            ((prin1-to-string +pepper-client-id+) :pepper)
            ("pepper" :pepper)
            (+turtle-client-id+ :turtle)
            ((prin1-to-string +turtle-client-id+) :turtle)
            ("turtle" :turtle)
            (otherwise nil)))
        (error-message
          "ERROR:
Usage: updateConnection(host, port, client-key)
Valid values for client-key are:
0 or \"pepper\" for pepper
2 or \"turtle\" for the turtlebot"))

    (when (not client-key)
      (return-from |updateObserverClient| -1))
    (when (stringp port)
      (setf port (parse-integer port)))
    
    (if (gethash client-key *clients*)
        ((lambda (client)
           (setf (client-host client) host)
           (setf (client-port client) port))
         (gethash client-key *clients*))
        (setf (gethash client-key *clients*)
              (make-client :host host :port port)))
    
    0))

     
     
