(defpackage :planning-common-package
  (:nicknames :common)
  (:use :cl :roslisp)
  (:export

   :+no-arm+
   :+left-arm+
   :+right-arm+
   :+both-arms+

   :+double+
   :+transform+

   :object-info
   :make-object-info
   :object-info-name
   :object-info-frame
   :object-info-timestamp
   :object-info-height
   :object-info-width
   :object-info-depth

   :service-run-pipeline
   :service-connect-frames

   :action-move-robot
   
   :prolog-disconnect-frames
   
   :ensure-node-is-running
   :make-param
   :get-transform-listener
   :extract-pose-from-transform
   :tf-pose->string
   :tf-lookup->string
   :file->string
   :split
   :strings->KeyValues
   :get-joint-config
   :get-controller-specs
   :seen-since
   :get-object-info))
