(in-package :plan-generator-package)

(defun service-generate-plan (domain task)
  "Return plan field of response message from calling service '/generate_plan'."
  (let ((srv "/generate_plan"))
    (if (not (wait-for-service srv 10))
        (ros-warn srv "Timed out waiting for service.")
        (with-fields
            (plan)
            (call-service srv
                          'plan_generator-srv:GeneratePlan
                          :domain domain
                          :task task)
          plan))))
