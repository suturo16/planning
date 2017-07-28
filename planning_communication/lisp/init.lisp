(in-package :planning-communication-package)

(defun setup-pepper-communication (execute-cb &key (pepper-ip "192.168.101.253") (pepper-port 8000))
  "Setup communcation with other agents.

EXECUTE-CB (function): Look at `make-command-executer's docstring for further information.
MY-IP (string):  IP of the system running planning.
PEPPER-IP (string): IP of Pepper.
PEPPER-PORT (int): RPC-Port of Pepper."
  ;; Listen to pepper_command topic
  (listen-for-commands execute-cb)

  ;; Initialize the RPC-server
  (init-rpc-server)

  ;; Put Pepper in our *clients* table.
  (|updateObserverClient| +pepper-client-id+ pepper-ip pepper-port)

  ;; Tell Pepper where we are.
  (update-connection-credentials :client :pepper))

  
