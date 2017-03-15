(in-package :pepper-communication-package)

(defun init-planning ()
  "Initialize everything planning needs to run."
  (start-ros-node "planning")

  ;; Initialize communication with pepper
  (setup-pepper-communication "192.168.101.97")
  
  ;; Initialize action client
  (pr2-do::setup-move-robot-client)
  
  ;; Let Perception see.
  ;;(pr2-do::run-full-pipeline)

  (pr2-do::service-run-pipeline "Knife"))

(defun setup-pepper-communication (my-ip &optional (pepper-ip "192.168.101.69") (pepper-port 8000))
  ;; Initialize the RPC-server
  (init-rpc-server)

  ;; Put Pepper in our *clients* table.
  (|updateObserverClient| +pepper-client-id+ pepper-ip pepper-port)

  ;; Tell Pepper where we are.
  (fire-rpc "updateObserverClient" pepper-ip pepper-port
            +pr2-client-id+ my-ip (get-local-port))

  ;; Listen to pepper_command topic
  (listen-to-pepper))
