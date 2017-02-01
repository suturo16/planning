(in-package :pepper-communication-package)

(defstruct client host port)

(defparameter *clients*  (alexandria:alist-hash-table '((:pepper . nil) (:turtle . nil))))

; Adjust the IP adress and port if necessary
(defparameter *host* "134.102.161.102")
(defparameter *port* 8080)

(defun update-connection-credentials (local-host remote-host remote-port)
  "Needs local ip address and remote ip adress and port.
Sends local host and port information of the calling machine to the addressee, for keep them up-to-date.
TODO: Retrieve IP address automatically. Import package ip-interfaces from external resources."
  (let ((local-port
          (nth-value 1
                     (sb-bsd-sockets:socket-name
                      (second (first s-xml-rpc::*server-processes*)))))) 
    (fire-rpc "updateConnection" remote-host remote-port local-host local-port 0)))

(defun fire-rpc (remote-function host port &rest arguments)  
  "Calls remote function of server with given hostname and port.
Arguments for the remote function can be added, if needed.
If host or port is nil, default is used."
  (when host
    (setf *host* host))
  (when port
    (setf *port* port))
  (s-xml-rpc:xml-rpc-call
   (apply 's-xml-rpc:encode-xml-rpc-call remote-function arguments)
   :host *host*
   :port *port*))
