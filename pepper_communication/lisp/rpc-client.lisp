; See API at https://common-lisp.net/project/s-xml-rpc/

(in-package :pepper-communication-package)

; Adjust the IP adress and port if necessary
(defparameter *host* "127.0.0.1")
(defparameter *port* 8000)

(defun fire-rpc (arguments &optional (remote-function "setStatus"))
  "Call this like (pcomm::fire-rpc \"Some great debug text\"). The default remote function publishes the text to the
 /server/status topic. Remember to have a rosnode startet within emacs."
  (s-xml-rpc:xml-rpc-call
   (s-xml-rpc:encode-xml-rpc-call remote-function arguments)
   :host *host*
   :port *port*))
