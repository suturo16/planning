(in-package :planning-communication-package)

(defun setup-pepper-communication (execute-cb my-ip &optional (pepper-ip "192.168.101.69") (pepper-port 8000))
  ;; Initialize the RPC-server
  (init-rpc-server)

  ;; Put Pepper in our *clients* table.
  (|updateObserverClient| +pepper-client-id+ pepper-ip pepper-port)

  ;; Tell Pepper where we are.
  (fire-rpc "updateObserverClient" pepper-ip pepper-port
            +pr2-client-id+ my-ip (get-local-port))

  ;; Listen to pepper_command topic
  (listen-for-commands execute-cb))
