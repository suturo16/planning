; See API at https://common-lisp.net/project/s-xml-rpc/
(in-package :pepper-communication-package)

(defstruct client host port)

(defparameter *clients* (alexandria:alist-hash-table '()))

; Adjust the IP adress and port if necessary
(defparameter *host* "134.102.157.234")
(defparameter *port* 8080)

(defun update-connection-credentials (local-host remote-host remote-port)
  "Needs local ip address and remote ip adress and port.
Sends local host and port information of the calling machine to the addressee, for keep them up-to-date.
TODO: Retrieve IP address automatically. Import package ip-interfaces from external resources."
  (let ((local-port
          (nth-value 1
                     (sb-bsd-sockets:socket-name
                      (second (first s-xml-rpc::*server-processes*)))))) 
    (fire-rpc "updateConnection" remote-host remote-port local-host local-port 1)))

(defun fire-rpc (remote-function host port &rest arguments)  
  "Calls remote functions of server with given hostname (ip address) and port.
Arguments can be added, if needed."
  (when host
    (setf *host* host))
  (when port
    (setf *port* port))
  (s-xml-rpc:xml-rpc-call
   (apply 's-xml-rpc:encode-xml-rpc-call remote-function arguments)
   :host *host*
   :port *port*))
