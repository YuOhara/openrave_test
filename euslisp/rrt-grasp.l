(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
(load "~/prog/euslib/jsk/searchlib.l")
(load "~/prog/euslib/demo/ohara/motion_plan/euslisp/rrt-jt-multigoal.l")
(load "~/prog/euslib/demo/ohara/motion_plan/euslisp/util.l")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-srvs "std_srvs")
(ros::roseus-add-srvs "openrave_test")


(defmethod rrt-jt-multigoal
  (:collision-free ;; need to implement this
   ()
   (not (check-collision :obj *tmp-world*))
   )
  )

(defmethod rrt-jt-multigoal
  (:distance (coords &key (sample-trans-abs nil))
    (let* ((relative-coords (send (send coords :copy-worldcoords) :inverse-transformation (send trans :copy-worldcoords)))
           (dist-sum 0))
      (setq dist-sum (+ dist-sum (* (elt weight 0) (norm (send relative-coords :pos)))))
      (setq dist-sum (+ dist-sum (* (elt weight 1) (norm (v* (matrix-column (send relative-coords :rot) 0) (float-vector 1 0 0))))))
      (let* (
             (quat (matrix2quaternion (send :relative-coords :rot)))
             (ve (float-vector (elt quat 1) (elt quat 2) (elt quat 3)))
             (f (sqrt (norm ve)))
             (theta
              (* 2.0 (/ (atan2 f (elt quat 1) f)))
              ))
        (setq dist-sum (+ dist-sum (* (elt weight 1)
                                      theta
                                      )))
        )
      ;; (sqrt dist-sum)
      dist-sum
      )
    )
  )

(setq *final-res* nil)

(defmethod rrt-jt-multigoal
  (:clear-func (s)
    (ros::ros-info "try clear func!")
    (send *robot* :rarm :angle-vector (send (elt (send s :solution-nodes) (- (length (send s :solution-nodes)) 1)) :state))
    (if
        (send *robot* :rarm :move-end-pos (float-vector 50 50 0))
        (return-from :clear-func t)
      (return-from :clear-func nil)
      )
    ;; (service-call-for-grasp s)
    )
  (:ik-func (robot arm coords)
    (send robot arm :inverse-kinematics coords :stop 5 :debug-view :no-message :warnp nil :move-target *hand-new-endcoords* :rotation-axis t)
    )
  (:collision-free ;; need to implement this
   ()
   (not (check-collision :obj *tmp-world*))
   )
  )

(defun service-call-for-grasp 
  (s)
  (ros::ros-info "try clear func!")
  (send *robot* :rarm :angle-vector (send (elt (send s :solution-nodes) (- (length (send s :solution-nodes)) 1)) :state))
  (send *irtviewer* :draw-objects)
  (setq grasp-coords (send (send robot arm :end-coords :copy-worldcoords) :transform (send (send *hand-coords* :copy-worldcoords) :inverse-transformation)))
  (setq grasp-coords (send (send (send trans-ground->kinfu :copy-worldcoords) :inverse-transformation)
                           :transform grasp-coords))
  (let*
      ((pose-msg (ros::coords->tf-pose grasp-coords))
       (req (instance openrave_test::GraspAssessRequest :init))
       res)
    (send req :pose_stamped (instance geometry_msgs::PoseStamped :init :pose pose-msg :header (instance std_msgs::Header :init :frame_id "ground" :stamp (ros::time-now))))
    (setq res (ros::service-call "/grasp_assess" req))
    (setq *res* res)
    (if (> (send res :assessment_point) 1e-6)
        (progn
          (setq *final-res* res)
          (send *robot* :rarm :inverse-kinematics (send (send (send trans-ground->kinfu :copy-worldcoords) :transform (ros::tf-pose->coords (send *final-res* :assessed_pose_stamped :pose))) :transform (send *hand-coords* :copy-worldcoords)))
          (send *irtviewer* :draw-objects)
          (return-from :clear-func t)
          )
      (return-from :clear-func nil)
      )
    )
  )

(defun init-robot ()
  (hrp2jsknts-init)
  (setq *robot* *hrp2jsknts*)
  (unless (boundp '*tfl*)
    (setq *tfl* (instance ros::transform-listener :init)))
  (setq *hand-coords*  (send (send (send (send *robot* :hand :rarm) :copy-worldcoords) :translate (float-vector 0 40 -160)) :transformation (send (send *robot* :rarm :end-coords) :copy-worldcoords)))
  (setq *hand-new-endcoords*
        (make-cascoords
         :coords
         ;; (send (send (send (send *robot* :hand :rarm) :copy-worldcoords) :translate (float-vector 0 40 -160)) :rotate (deg2rad 50) :x)
         (send (send *robot* :hand :rarm) :copy-worldcoords)
         :parent (send (send *robot* :rarm :end-coords) :parent)))
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :hand :rarm :hook-pose)
  )


(defun callback (msg)
  (setq *msg* msg)
  (callback2 *msg*)
  )

(defun go-pos-coords (coords)
  (send *robot* :fix-leg-to-coords (make-coords :pos (float-vector (- (elt (send coords :pos) 0) 550) 0 0)))
  )

(defun callback2 (msg)
  ;; transform to robot frame
  ;; (send *robot* :fix-leg-to-coords (make-coords))
  ;; (send *robot* :fix-leg-to-coords (make-coords :pos (float-vector 200 0 0)))
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
  (setq *wgrs* nil)
  (dolist (obj (send msg :poses))
    (let ((trs
           (send (send trans-ground->kinfu :copy-worldcoords)
                 :transform (ros::tf-pose->coords  obj))))
      (send trs :translate (float-vector 0 -50 50)) ;; offset for avoid collision
      ;; (send trs :rotate (deg2rad -90) :y)
      ;; (send trs :rotate (deg2rad -180) :y)
      (let
          ((trs-0 (send trs :copy-worldcoords)))
        (push trs-0 lst)
        (push
         (instance workspace-goal-region :init trs-0 :min-range (float-vector 0 0 0 0 0 0) :max-range (float-vector 0 0 0 0 0 0))
         *wgrs*)
        )
        ))
  (go-pos-coords (elt lst 0))
  (send *robot* :reset-manip-pose)
  ;; (send *robot* :fix-leg-to-coords (make-coords))
  ;; (send *robot* :fix-leg-to-coords (make-coords :pos (float-vector 200 200 0)))
  (setq *solver*
        (instance rrt-jt-multigoal :init
                  *robot*
                  :rarm
                  *wgrs*
                  :stop 2000
                  :end-coords *hand-new-endcoords*
                  ))
  ;; (setf (send *robot* :rarm :end-coords) *hand-new-endcoords*)
  (setq *ret* (send *solver* :solve))
  )

(defun show-motion
  ()
  (dolist (state (send-all (send *ret* :solution-nodes) :state))
    (send *robot* :rarm :angle-vector state)
    (send *irtviewer* :draw-objects)
    (read-line))
  )

(defun do-motion
  ()
  (open-hand)
  (send *ri* :angle-vector (send *robot* :reset-manip-pose) 10000)
  (send *ri* :wait-interpolation)
  (send *ri* :go-pos (* (elt (send (send *robot* :foot-midcoords) :pos) 0) 0.001) 0 0)
  (dolist (state (send-all (send *ret* :solution-nodes) :state))
    (send *robot* :rarm :angle-vector state)
    (send *irtviewer* :draw-objects)
    (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    )
  ;; offset
  (send *robot* :rarm :move-end-pos (float-vector 50 50 0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  (close-hand)
  )
(defun try-move
  (angle-vector)
  (send *ri* :angle-vector angle-vector 10000)
  (dotimes (i 20)
    (unix::usleep 500000)
    (call-image-save)
    )
  ;; save bmf file
  (save-bmf-file)
  (unix::system "cp ~/.ros/left00*.jpg $(rospack find openrave_test)/moseg/TrainingSet/Data/fromrobot/")
  (unix::system "mogrify -format ppm $(rospack find openrave_test)/moseg/TrainingSet/Data/fromrobot/left*")
  ;; call moseg
  (call-moseg)
  )
(defun call-image-save 
  ()
  (let ((req (instance std_srvs::EmptyRequest :init)) res)
    (setq res (ros::service-call "/save_image_for_moseg/save" req))
    )
  )
(defun save-bmf-file
  ()
  ;; dummy
  ;; file size is given
  )
(defun open-hand
  ()
  (send *robot* :hand :rarm :angle-vector (float-vector 90.0 90.0 0.0 0.0 -40.0 -40.0))
  (hand-model2real)
  )
(defun close-hand
  ()
  (send *robot* :hand :rarm :r_thumbcm_p :joint-angle -30)
  (hand-model2real)
  (send *robot* :hand :rarm :r_middlepip_r :joint-angle 110)
  (hand-model2real)
  (send *robot* :hand :rarm :r_indexmp_r :joint-angle 30)
  (hand-model2real)
  (send *robot* :hand :rarm :r_indexpip_r :joint-angle 60)
  (hand-model2real)
  )


(defun hand-model2real (&key (time 500) (wait? t))
  (send *ri* :hand-angle-vector (apply #'concatenate float-vector (send *robot* :hand :arms :angle-vector)) time)
  (when wait? (send *ri* :hand-wait-interpolation)))

  
(defun call-moseg
  ()
  (cd "/home/leus/ros/indigo/src/openrave_test/moseg") ;; need to change dir
  (unix::system "rm -rf $(rospack find openrave_test)/moseg/TrainingSet/Data/fromrobot/FlowFiles")
  (unix::system "$(rospack find openrave_test)/moseg/MoSeg $(rospack find openrave_test)/moseg/filestructureTrainingSet.cfg fromrobot 0 20 4 20")
  )

(ros::roseus "sample-grasp-node")
(init-robot)
(ros::subscribe "/grasp_caluculation_result" geometry_msgs::PoseArray #'callback)
