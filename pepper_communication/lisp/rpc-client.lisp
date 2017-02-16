(in-package :pepper-communication-package)

                                        ; Adjust the IP adress and port if necessary
(defparameter *host* "134.102.161.102")
(defparameter *port* 8080)

(defun update-connection-credentials (local-host &optional &key (client-id 1) remote-host remote-port client)
  "LOCAL-HOST: The local computers IP address.
CLIENT-ID: Id of the calling system. 0 = pepper, 1 = PR2, 2 = turtle
REMOTE-HOST: Address of the server to call.
REMOTE-PORT: Port of the server to call.
CLIENT: Set to :pepper or :turtle instead of setting host and port manually, to retrieve information from the clients list.

Sends local host and port information of the calling machine to the addressee, to keep them up-to-date.
TODO: Retrieve IP address automatically. Import package ip-interfaces from external resources."
  
  (unless (and remote-host remote-port)
    (setf remote-host (client-host (gethash client *clients*)))
    (setf remote-port (client-port (gethash client *clients*))))
  
  (fire-rpc "updateObserverClient" remote-host remote-port client-id local-host (get-local-port)))

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
  (when host
    (setf *host* host))
  (when port
    (setf *port* port))
  (s-xml-rpc:xml-rpc-call
   (apply 's-xml-rpc:encode-xml-rpc-call remote-function args)
   :host *host*
   :port *port*))

(defun get-local-port ()
  "Returns the local port of the server."
  (nth-value 1
             (sb-bsd-sockets:socket-name
              (second (first s-xml-rpc::*server-processes*)))))

