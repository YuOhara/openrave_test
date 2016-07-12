(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
(load "~/prog/euslib/jsk/searchlib.l")
(load "~/prog/euslib/demo/ohara/motion_plan/euslisp/rrt-jt-multigoal.l")
(load "~/prog/euslib/demo/ohara/motion_plan/euslisp/util.l")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-srvs "std_srvs")

(defmethod rrt-jt-multigoal
  (:collision-free ;; need to implement this
   ()
   (not (check-collision :obj *tmp-world**))
   )
  )

(defmethod rrt-jt-multigoal
  (:distance (coords &key (sample-trans-abs nil))
    (let* ((relative-coords (send (send coords :copy-worldcoords) :inverse-transformation (send trans :copy-worldcoords)))
           (dist-sum 0))
      (setq dist-sum (+ dist-sum (* (elt weight 0) (norm (send relative-coords :pos)))))
      (setq dist-sum (+ dist-sum (* (elt weight 1) (norm (v* (matrix-column (send relative-coords :rot) 0) (float-vector 1 0 0))))))
      (sqrt dist-sum)
      )
    )
  )

(defun clear-func (s)
  (send *robot* :rarm :angle-vector (elt (send s :solution-nodes) (- (length (send s :solution-nodes)) 1)))
  (setq grasp-coords (send *hand-new-endcoords* :copy-worldcoords))
  (ros::coords->tf-pose  obj)
  )

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
  (send *robot* :fix-leg-to-coords (make-coords))
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