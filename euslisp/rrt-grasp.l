(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
(load "~/prog/euslib/jsk/searchlib.l")
(load "~/prog/euslib/demo/ohara/motion_plan/euslisp/rrt-jt-multigoal.l")
(load "~/prog/euslib/demo/ohara/motion_plan/euslisp/util.l")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-srvs "std_srvs")

(defun init-robot ()
  (hrp2jsknts-init)
  (setq *robot* *hrp2jsknts*)
  (unless (boundp '*tfl*)
    (setq *tfl* (instance ros::transform-listener :init)))
  (setq *hand-coords*  (send (send (send (send *robot* :hand :rarm) :copy-worldcoords) :translate (float-vector 0 40 -150)) :transformation (send (send *robot* :rarm :end-coords) :copy-worldcoords)))
  (setq *hand-new-endcoords*
        (make-cascoords
         :coords
         (send (send (send *robot* :rarm :end-coords) :copy-worldcoords) :transform (send *hand-coords* :inverse-tranformation) :local)
         :parent (send (send *robot* :rarm :end-coords) :parent)))
  )


(defun callback (msg)
  (setq *msg* msg)
  (callback2 *msg*)
  )

(defun callback2 (msg)
  ;; transform to robot frame
  (send *robot* :fix-leg-to-coords (make-coords))
  (setq trans-ground->kinfu
        (send *tfl* :lookup-transform
              "/ground"
              (send msg :header :frame_id)
              (ros::time 0)))
  ;; (ros::service-call "/file_getter" (instance std_srvs::Emptyrequest :init))
  (load "/home/leus/.ros/tmp_model.l")
  (tmp-world)
  (send *tmp-world* :newcoords (send trans-ground->kinfu :copy-worldcoords))
  (objects (list *robot* *tmp-world*))
  (setq lst nil)
  (let (ik-result)
    (dolist (obj (send msg :poses))
      (let ((trs
             (send (send trans-ground->kinfu :copy-worldcoords)
                   :transform (ros::tf-pose->coords  obj))))
        (push trs lst)
        ))
    )
  )

(ros::roseus "sample-grasp-node")
(init-robot)
(ros::subscribe "/grasp_caluculation_result" geometry_msgs::PoseArray #'callback)
