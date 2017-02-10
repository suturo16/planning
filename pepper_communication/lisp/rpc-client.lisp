(in-package :pepper-communication-package)

; Adjust the IP adress and port if necessary
(defparameter *host* "134.102.161.102")
(defparameter *port* 8080)

(defun update-connection-credentials (local-host remote-host remote-port client-id)
  "Needs local ip address and remote ip adress and port.
Sends local host and port information of the calling machine to the addressee, for keep them up-to-date.
TODO: Retrieve IP address automatically. Import package ip-interfaces from external resources."
  (let ((local-port (get-local-port))) 
    (fire-rpc
     "updateObserverClient"
     remote-host
     remote-port
     local-host
     local-port
     client-id)))

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
