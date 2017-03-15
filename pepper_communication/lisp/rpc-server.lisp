(in-package :pepper-communication-package)

;; Active clients saved in a hash map. Update map and use those credentials for remote calls to pepper and turtle.
(defstruct client host port)
(defparameter *clients*  (alexandria:alist-hash-table '((:pepper . nil) (:turtle . nil))))

;; Client ids for mapping to keys in client hash map. See function |updateObserverClient|.
(alexandria:define-constant +pepper-client-id+ 0)
(alexandria:define-constant +pr2-client-id+ 1)
(alexandria:define-constant +turtle-client-id+ 2)


(defun init-rpc-server (&optional (restart-rosnode nil))
  "Starts and initializes the RPC server and rosnode, if needed or wanted.
RESTART-ROSNODE: Ste to T, if you want to force-start a new rosnode."
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
  "Waits 3 seconds, before responding. For debugging pusposes.
DUMMY: Unused parameter to prevent issues with calls without parameters."
  (sleep 3))


(defun |cutCake| (dummy)
  "Calls the do-function with 'cut-cake'.
DUMMY: Unused parameter to prevent issues with calls without parameters."
  (|do| "cut cake"))


(defun |do| (command)
  "Publishes to the command listeners topic and responds with the amount of due tasks.
0 indicates immediate execution of given task.
COMMAND: The message to publish onto pepper_command"
  (let ((pub (advertise "/pepper_command" "std_msgs/String")))
    (if  (not (eq (roslisp:node-status) :SHUTDOWN))
         (progn (publish-msg pub :data command)
                (|stressLevel| "asdf"))
         -1)))


(defun |stressLevel| (dummy)
  "Returns the current stress level, represented by the length of tasks in task-buffer,
or -1 if the subscriber is unavailable.
DUMMY: Unused parameter to prevent issues with calls without parameters."
  (when (or
         (not *pepper-subscriber*)
         (not (roslisp::thread-alive-p (roslisp::topic-thread (roslisp::subscriber-subscription *pepper-subscriber*)))))
    (return-from |stressLevel| -1))
  
  (roslisp-queue:queue-size (roslisp::buffer (roslisp::subscriber-subscription *pepper-subscriber*))))


(defun |nextTask| ()
  "Returns the identifier of the next task, as is in the commands list."
  "Not implemented!")


(defun |updateObserverClient| (client-id host port)
  "Update clients' information about host and port, using the client id as primary key.
CLIENT-ID: Id of the calling client. See id definition of the machines above.
HOST: IP address of the calling machine.
PORT: Port of the calling machine."
  (let ((client-key
          (alexandria:switch (client-id :test #'equal)
            (+pepper-client-id+ :pepper)
            ((prin1-to-string +pepper-client-id+) :pepper)
            ("pepper" :pepper)
            (+turtle-client-id+ :turtle)
            ((prin1-to-string +turtle-client-id+) :turtle)
            ("turtle" :turtle)
            (otherwise nil))))

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

     
     
