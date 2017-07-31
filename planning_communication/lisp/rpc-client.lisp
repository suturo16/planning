(in-package :planning-communication-package)

(defun update-connection-credentials (&key (client-id 1) remote-host remote-port client)
  "CLIENT-ID: Id of the calling system. 0 = pepper, 1 = PR2, 2 = turtle
REMOTE-HOST: Address of the server to call.
REMOTE-PORT: Port of the server to call.
CLIENT: Set to :pepper or :turtle instead of setting host and port manually, to retrieve information from the clients list.
Sends local host and port information of the calling machine to the addressee, to keep them up-to-date."
  (unless (and remote-host remote-port)
    (setf remote-host (client-host (gethash client *clients*)))
    (setf remote-port (client-port (gethash client *clients*))))
  (fire-rpc "updateObserverClient" remote-host remote-port client-id (get-local-ip) (get-local-port)))

(defun fire-rpc-to-client (client remote-function &rest args)
  "CLIENT: The id of the client as saved in *clients*.
REMOTE-FUNCTION: Function name to call on remote host.
ARGS: Arguments for the remote function.

Call function on remote host."
  (if (gethash client *clients*)
      (apply 'fire-rpc remote-function
             (client-host (gethash client *clients*))
             (client-port (gethash client *clients*))
             args)
      (format t "No credentials for client ~a found." client)))

(defun fire-rpc (remote-function host port &rest args)  
  "Calls remote function of server with given hostname and port.
Arguments for the remote function can be added, if needed.
If host or port is nil, default is used."
  (s-xml-rpc:xml-rpc-call
   (apply 's-xml-rpc:encode-xml-rpc-call remote-function args)
   :host host
   :port port))

(defun get-local-ip ()
  "Returns the IP of the current connection, NIL if there is none."
  (let* ((eth-interfaces
           (ip-interfaces:get-ip-interfaces-by-flags '(:iff-up :iff-running :iff-broadcast)))
         (eth-adress
           (ip-interfaces:ip-interface-address (first eth-interfaces))))
    (when eth-adress
      (format nil "~{~a~^.~}" (map 'list #'identity eth-adress)))))

(defun get-local-ip-by-name (if-name)
  "Returns the IP of the interface with given name, NIL if there is none of this name."
  (let ((interfaces
          (ip-interfaces:get-ip-interfaces))
        (found-interface nil))
    (loop for interface in interfaces
          do (when (equal (ip-interfaces:ip-interface-name interface) if-name)
               (setf found-interface interface)))
    (when found-interface
      (format nil "~{~a~^.~}"
              (map 'list #'identity (ip-interfaces:ip-interface-address found-interface))))))

(defun get-local-port ()
  "Returns the local port of the server."
  (when s-xml-rpc::*server-processes*
    (nth-value 1
               (sb-bsd-sockets:socket-name
                (second (first s-xml-rpc::*server-processes*))))))

(defun notify-order-ready (customer-id)
  "Sends notification to pepper about a finished order."
  (let ((json-string (cl-json:encode-json-alist-to-string
                      `(("guestId" . ,customer-id)
                        ("return" ("type" . "complete")
                                  ("success" . "1"))))))
    (when (gethash :pepper *clients*)
      (fire-rpc-to-client :pepper "new_notify" json-string))))
