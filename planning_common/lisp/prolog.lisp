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

(defun prolog-guest-info (guest-id &rest request-arguments) 
  "Query prolog with <need functionname here> to update and retrieve user info."
  (json-prolog:prolog `("need_real_function_name_here"
                        ,guest-id ,request-arguments) :lispify T :package :common))
