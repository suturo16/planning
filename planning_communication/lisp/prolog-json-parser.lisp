(in-package :planning-communication-package)

(defparameter *json-example* (cl-json:encode-json-alist-to-string
                              '(("guestId" . "1") ("return" . (("type" . "setCake") ("success" . "1") ("location" . "table1"))))))

(defparameter *json-assoc* (cl-json:encode-json-alist-to-string
                          '((:guestId . "1") (:query . ((:type . "setCake") (:amount . "1") (:name . "arthur"))))))

(defparameter *without-quotes* "{guestId:1,query:{type:setCake,amount:1,guestName:arthur}}")


(defun handle-knowledge-update (json-string)
  "Starts a new thread that updates the knowledgebase with given json string.
The json structure is defined here:
https://docs.google.com/document/d/1wCUxW6c1LhdxML294Lvj3MJEqbX7I0oGpTdR5ZNIo_w"
  ;; (unless (member guest-id common:*guests* :test #'equal)
  ;;   (nconc common:*guests* '(guest-id)))
  (sb-thread:make-thread (lambda ()
                           (sb-thread:with-mutex ((get-prolog-mutex))
                             (common:prolog-assert-dialog-element json-string))
                           (common:say "Thank you for the information.")
                           
                           (when (gethash :pepper *clients*)
                             (fire-rpc-to-client :pepper "notify")))))

(defun compose-reponse (json-string)
  "Check the type of query in the json and decide, what to return to pepper."
  (let* ((json-object (cl-json:decode-json-from-string json-string))
         (query-type (last (assoc :type (last (assoc :query json-object)))))
         (response-a-list '(("guestId" . (cdr (assoc :guest-id json-object))))))
    ))

(defun handle-get-customer-info (&optional customer-id)
  "Queries the knowledgebase for information about a specific customer."
  (let ((raw-customer-response (common:prolog-get-customer-infos customer-id))
        (raw-order-response (common:prolog-get-open-orders-of customer-id))
        name place item amount delivered)
    (when raw-customer-response
      (cut:with-vars-bound
          (common::?Name common::?Place)
          raw-customer-response
        (setf name common::?Name)
        (setf place common::?Place)))
    (when raw-order-response
      (cut:with-vars-bound
          (common::?Item common::?Amount common::?Delivered)
          raw-order-response
        (setf item common::?Item)
        (setf amount common::?Amount)
        (setf delivered common::?Delivered)))
    (cl-json:encode-json-alist-to-string
     (guest-info-arguments->a-list customer-id name place amount delivered))))

(defun handle-get-all-customer-info ()
  "Queries the knowledgebase for the whole list of cstomer infos and parses the list to json."
  (let ((raw-customer-response (common:prolog-get-customer-infos))
        (raw-order-response (common:prolog-get-open-orders-of))
         
        customer-id name place item amount delivered)
    (concatenate 'string
                 "["
                 (reduce (lambda (first next) (concatenate 'string first next))
                         (loop for i from 0 to (- (length raw-customer-response) 1)
                               do (when raw-customer-response
                                    (cut:with-vars-bound
                                        (common::?CustomerID common::?Name common::?Place)
                                        (nth i raw-customer-response)
                                      (setf customer-id common::?CustomerID)
                                      (setf name common::?Name)
                                      (setf place common::?Place)))
                                  (when raw-order-response
                                    (cut:with-vars-bound
                                        (common::?Item common::?Amount common::?Delivered)
                                        (nth i raw-order-response)
                                      (setf item common::?Item)
                                      (setf amount common::?Amount)
                                      (setf delivered common::?Delivered)))
                               collect (cl-json:encode-json-alist-to-string
                                        (guest-info-arguments->a-list customer-id name place amount delivered))))
                 "]")))

(defun guest-info-arguments->a-list (customer-id name place amount delivered)
  `(("guestId" . ,(format nil "~a" customer-id))
    ("return" . (("type" . "getGuestInfo")
                 ("name" . ,(format nil "~a" name))
                 ("location" . ,(format nil "~a" place))
                 ("total" . ,(format nil "~a" amount))
                 ("delivered" . ,(format nil "~a" delivered))))))

(defun format-nil (destination control-string &rest args)
  "Uses the standard `format' function but maps NIL arguments to empty strings instead."
  (format destination control-string (values-list (map 'list (lambda (x) (if x x "")) args))))
