(in-package :planning-common-package)

(alexandria:define-constant +knowrob-iri-prefix+ "http://knowrob.org/kb/knowrob.owl#" :test #'string=)


(defun prolog-get-object-info (&optional (type NIL) (name NIL))
  "Query prolog with 'get_object_infos' and TYPE and optionally NAME as bound variables."
  (cut:lazy-car (json-prolog:prolog
                 `("get_object_infos"
                   ,(if name (format nil "~a~a" +knowrob-iri-prefix+ name) `?name)
                   ?frame
                   ,(if type type `?type)
                   ?timestamp
                   ?pose
                   ?width
                   ?height
                   ?depth) :lispify T :package :common)))


(defun prolog-seen-since (name frame-id timestamp)
  "Query prolog with 'seen_since(NAME, FRAME_ID, TIMESTAMP)'.
Return an empty list if object NAME with FRAME-ID was seen since TIMESTAMP.
Return nil otherwise."
  (json-prolog:prolog
                 `("seen_since"
                  ,(format nil "~a~a" +knowrob-iri-prefix+ name)
                  ,frame-id ,timestamp) :lispify T :package :common))

(defun prolog-connect-frames (parent-frame-id child-frame-id)
  "connect frames through prolog."
  (json-prolog:prolog
                 `("connect_frames"
                   ,parent-frame-id ,child-frame-id) :lispify T :package :common))

(defun prolog-disconnect-frames (parent-frame-id child-frame-id)
  "Query prolog with 'disconnect_frames(PARENT-FRAME-ID, CHILD-FRAME-ID)'.
Disconnects the frames PARENT-FRAME-ID and CHILD-FRAME-ID."
  (json-prolog:prolog
                 `("disconnect_frames"
                   ,parent-frame-id ,child-frame-id) :lispify T :package :common))

(defun prolog-assert-dialog-element (json-string) 
  "Query prolog with 'assert_dialog_element' to update guest info. Remove all double quotes in the json object."
  (cut:lazy-car (json-prolog:prolog-simple
                 (format nil "assert_dialog_element(~a)" (remove #\" json-string)) :lispify T :package :common)))

(defun prolog-get-customer-infos (&optional customer-id)
  "Query prolog with 'get_customer_infos' to get the name and place of a specific guest. Provides all guest infos per default"
  (cut:lazy-car (json-prolog:prolog
                 `("get_customer_infos"
                   ,(if customer-id customer-id `?CustomerID)
                   ?Name
                   ?Place) :lispify T :package :common)))

(defun prolog-get-open-orders-of (&optional customer-id)
  "Query prolog with 'get_guest_info' to get the delivery item and the amount for a specific guest. Provides all orders per default."
  (cut:lazy-car (json-prolog:prolog
                 `("get_open_orders_of"
                   ,(if customer-id customer-id `?CustomerID)
                   ?Item
                   ?Amount
                   ?Delivered) :lispify T :package :common)))

(defun prolog-get-free-table ()
  "Query prolog with 'get_free_table' to get the name of a free table."
  (cut:lazy-car (json-prolog:prolog
                 `("get_free_table"
                   ?NameOfFreeTable) :lispify T :package :common)))

(defun prolog-set-delivered-amount (customer-id amount)
  "Query prolog with 'increase_deliver_amount' to increase the item amount for an order."
  (cut:lazy-car (json-prolog:prolog
                 `("set_delivered_amount"
                   ,customer-id
                   ,amount) :lispify T :package :common)))

(defun prolog-increase_delivered_amount (customer-id)
  "Query prolog with 'increase_deliver_amount' to increase the item amount for an order."
  (cut:lazy-car (json-prolog:prolog
                 `("increase_delivered_amount"
                   ,customer-id) :lispify T :package :common)))
