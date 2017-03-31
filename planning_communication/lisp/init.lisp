(in-package :planning-communication-package)

(defun setup-pepper-communication (execute-cb my-ip &optional (pepper-ip "192.168.101.69") (pepper-port 8000))
  "Setup communcation with other agents.

EXECUTE-CB (function): Look at `make-command-executer's docstring for further information.
MY-IP (string):  IP of the system running planning.
PEPPER-IP (string): IP of Pepper.
PEPPER-PORT (int): RPC-Port of Pepper."
  ;; Initialize the RPC-server
  (init-rpc-server)

  ;; Put Pepper in our *clients* table.
  (|updateObserverClient| +pepper-client-id+ pepper-ip pepper-port)

  ;; Tell Pepper where we are.
  (fire-rpc "updateObserverClient" pepper-ip pepper-port
            +pr2-client-id+ my-ip (get-local-port))

  ;; Listen to pepper_command topic
  (listen-for-commands execute-cb))
