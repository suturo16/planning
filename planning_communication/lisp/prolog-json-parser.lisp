(in-package :planning-communication-package)

(defparameter *json-example* (cl-json:encode-json-alist-to-string
                              '(("guestId" . "1") ("return" ("type" . "setCake") ("success" . "1") ("location" . "table1")))))

(defparameter *json-assoc* (cl-json:encode-json-alist-to-string
                            '((:guestId . "1") (:query (:type . "setCake") (:amount . "1") (:name . "arthur")))))


(defparameter *json-assoc-d* (cl-json:encode-json-alist-to-string
                              `(("guestId" . "1") ("query" ("type" . "setLocation") ("tableId" . "table1")))))

(defparameter *json-set-cake* (cl-json:encode-json-alist-to-string
                              `(("guestId" . "1") ("query" ("type" . "setCake") ("amount" . "1") ("guestName" . "arthur")))))

(defparameter *json-set-cake-unescaped* "{\"guestId\":\"1\",\"query\":{\"type\":\"setCake\",\"amount\":\"1\",\"guestName\":\"arthur\"}}")

(defparameter *json-set-cake-unescaped2* "{\"guestId\":\"2\",\"query\":{\"type\":\"setCake\",\"amount\":\"1\",\"guestName\":\"arthur\"}}")

(defparameter *without-quotes* "{guestId:1,query:{type:setCake,amount:1,guestName:arthur}}")


(defun handle-knowledge-update (json-string)
  "Starts a new thread that updates the knowledgebase with given json string.
The json structure is defined here:
https://docs.google.com/document/d/1wCUxW6c1LhdxML294Lvj3MJEqbX7I0oGpTdR5ZNIo_w"
  (common:prolog-assert-dialog-element json-string)
  (cl-json:encode-json-alist-to-string (compose-reponse json-string))
  ;; (sb-thread:make-thread (lambda ()
  ;;                          (sb-thread:with-mutex ((get-prolog-mutex))
  ;;                            (common:prolog-assert-dialog-element json-string))
  ;;                          (common:say "Thank you for the information.")
  ;;                          (compose-reponse json-string)
  ;;                          ))
  )

(defun handle-get-customer-info (&optional customer-id)
  "Starts a new thread to query the knowledgebase for information about a specific customer."
  (let ((response (thread-get-customer-info
                   (if customer-id customer-id (common:get-current-order)))))
    response))

(defun handle-get-all-customer-info ()
  "Starts a new thread to query the knowledgebase for information about all customers."
  (let ((response (thread-get-all-customer-info)))
    response))

(defun compose-reponse (json-string)
  "Check the type of query in the json and decide, what to return to pepper."
  (let* ((json-object (cl-json:decode-json-from-string json-string))
         (query-type (alexandria:assoc-value (alexandria:assoc-value  json-object :query) :type))
         (customer-id (alexandria:assoc-value json-object :guest-id)))
    (if (equal query-type "setCake")
        (let ((place (common:get-free-table)))
          (assign-guest-to-place customer-id place)
          `(("guestId" . ,customer-id)
            ("return" ("type" . ,query-type)
                      ("success" . "1")
                      ("tableId" . ,place))))
        `(("guestId" . ,customer-id)
            ("return" ("type" . ,query-type)
                      ("success" . "1"))))))

(defun thread-get-customer-info (customer-id)
  (let ((raw-customer-response (car (common:prolog-get-customer-infos customer-id)))
        (raw-order-response (car (common:prolog-get-open-orders-of customer-id)))
        name place item amount delivered)
    (when raw-customer-response
      (cut:with-vars-bound
          (common::|?Name| common::|?Place|)
          raw-customer-response
        (setf name common::|?Name|)
        (setf place (place->string common::|?Place|))))
    (when raw-order-response
      (cut:with-vars-bound
          (common::|?Item| common::|?Amount| common::|?Delivered|)
          raw-order-response
        (setf item common::|?Item|)
        (setf amount common::|?Amount|)
        (setf delivered common::|?Delivered|)))
    (cl-json:encode-json-alist-to-string
     (guest-info-arguments->a-list customer-id name place amount delivered))))

(defun thread-get-all-customer-info ()
  "Queries the knowledgebase for the whole list of cstomer infos and parses the list to json."
  (let ((raw-customer-response (common:prolog-get-customer-infos))
        (raw-order-response (common:prolog-get-open-orders-of))
        customer-id name place item amount delivered)
    (concatenate 'string
                 "["
                 (reduce (lambda (first next) (concatenate 'string first "," next))
                         (loop for i from 0 to (- (length raw-customer-response) 1)
                               do (when raw-customer-response
                                    (cut:with-vars-bound
                                        (common::|?CustomerID| common::|?Name| common::|?Place|)
                                        (nth i raw-customer-response)
                                      (setf customer-id common::|?CustomerID|)
                                      (setf name common::|?Name|)
                                      (setf place (place->string common::|?Place|))))
                                  (when raw-order-response
                                    (cut:with-vars-bound
                                        (common::|?Item| common::|?Amount| common::|?Delivered|)
                                        (nth i raw-order-response)
                                      (setf item common::|?Item|)
                                      (setf amount common::|?Amount|)
                                      (setf delivered common::|?Delivered|)))
                               collect (cl-json:encode-json-alist-to-string
                                        (guest-info-arguments->a-list customer-id name place amount delivered))))
                 "]")))

(defun assign-guest-to-place (customer-id place)
  (let ((id (if (integerp customer-id) (write-to-string customer-id) customer-id)))
    (common:prolog-assert-dialog-element (cl-json:encode-json-alist-to-string
                              `(("guestId" . ,id) ("query" ("type" . "setLocation") ("tableId" . ,place)))))))

(defun guest-info-arguments->a-list (customer-id name place amount delivered)
  `(("guestId" . ,(format nil "~a" customer-id))
    ("return" ("type" . "getGuestInfo")
              ("name" . ,(format nil "~a" name))
              ("location" . ,(format nil "~a" place))
              ("total" . ,(format nil "~a" amount))
              ("delivered" . ,(format nil "~a" delivered)))))

(defun format-nil (destination control-string &rest args)
  "Uses the standard `format' function but maps NIL arguments to empty strings instead."
  (format destination control-string (values-list (map 'list (lambda (x) (if x x "")) args))))

(defun place->string (knowrob-place)
  (string-downcase (subseq (symbol-name knowrob-place) (1+ (position #\# (symbol-name knowrob-place) :from-end t)))))
