(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.k")
(ros::roseus-add-msgs "geometry_msgs")

(defun init-robot ()
  (hrp2jsknts-init)
  (setq *robot* *hrp2jsknts*)
  (unless (boundp '*tfl*)
    (setq *tfl* (instance ros::transform-listener :init)))
  )

(defun callback (msg)
  ;; transform to robot frame
  (send *robot* :fix-leg-to-coords (make-coords))
  (setq trans-ground->kinfu
        (send *tfl* :lookup-transform
              "/ground"
              (send *msg* :header :frame_id)
              (ros::time 0)))
  (let (lst ik-result)
    (dolist (obj (send *msg* :poses))
      (let ((trs
             (send (send trans-base->table :copy-worldcoords)
                   :transform (ros::tf-pose->coords  obj))))
        (setf (get trs :type) (send obj :type))
        (push trs lst)
        ))
    (block :solve-ik
      (dolist (pose lst)
        (setq ik-result (send *robot* :rarm :inverse-kinematics pose))
        (if ik-result
            (return-from :solve-ik nil)
          )
        )
      )
    (return ik-result)
    )
  )

(ros::roseus "sample-grasp-node")
(init-robot)
(ros::subscribe "/grasp_caluculation_result" posedetection_msgs::ObjectDetection
                #'detect-callback)
