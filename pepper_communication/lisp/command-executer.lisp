(in-package :pepper-communication-package)

(defun execute-pepper-command (command)
  "Execute the given command."
  
  (with-fields (data)
      command 
    (plan-execution-package::execute data))
  
  (when (gethash :pepper *clients*)
      (fire-rpc-to-client :pepper "notify")))

(defun init-planning (my-ip &optional (pepper-ip "192.168.101.69") (pepper-port 8000))
  "Initialize everything planning needs to run."
  ;; Start a node.
  (start-ros-node "planning")
  ;; Initialize the RPC-server
  (init-rpc-server)
  ;; Put Pepper in our *clients* table.
  (|updateObserverClient| +pepper-client-id+ pepper-ip pepper-port)
  ;; Tell Pepper where we are.
  (fire-rpc "updateObserverClient" pepper-ip pepper-port
            +pr2-client-id+ my-ip (get-local-port))
  ;; Let Perception see.
  ;;(pr2-do::run-full-pipeline)
  (pr2-do::service-run-pipeline "Knife"))

(defun init-planning-without-pepper ()
  (start-ros-node "planning")
  (pr2-do::setup-move-robot-client)
 ;;(pr2-do::run-full-pipeline)
  )
