(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
(load "~/prog/euslib/jsk/searchlib.l")
(load "~/prog/euslib/demo/ohara/motion_plan/euslisp/rrt-jt-multigoal.l")
(load "~/prog/euslib/demo/ohara/motion_plan/euslisp/util.l")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-srvs "std_srvs")
(ros::roseus-add-srvs "openrave_test")
(ros::roseus-add-msgs "openrave_test")
(load "package://openrave_test/euslisp/utility.l")


(defmethod rrt-jt-multigoal
  (:collision-free ;; need to implement this
   ()
   (not (check-collision :obj *tmp-world* 
                         :link-list (send *robot* arm :links)
                         ;; :hand? t
                         )
        )
   )
  )

(defmethod rrt-jt-multigoal
  (:distance (coords &key (sample-trans-abs nil))
    (let* ((relative-coords (send (send coords :copy-worldcoords) :inverse-transformation (send trans :copy-worldcoords)))
           (dist-sum 0))      (setq dist-sum (+ dist-sum (* (elt weight 0) (norm (send relative-coords :pos)))))
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
    )
  (:ik-func (robot arm coords)
    (send robot arm :inverse-kinematics coords :stop 5 :debug-view :no-message :warnp nil :move-target (send *robot* arm :end-coords) :rotation-axis t)
    )
  )

(defun move-robot (coords)
  (send *robot* :fix-leg-to-coords (send (send (send *robot* :foot-midcoords) :copy-worldcoords) :transform coords))
  )
(defun show-pose (i)
  (let ((foot-coords (send *robot* :foot-midcoords)))
    (progn (send *robot* :angle-vector (elt *ik-result* i)) (send *robot* :fix-leg-to-coords foot-coords))
    (send *irtviewer* :draw-objects)
    )
  )
(defun all-ik ()
  (setq *ik-result* nil)
  (let ((foot-coords (send *robot* :foot-midcoords)))
    (dolist (pose lst)
      ;; (setq ik-result (send *robot* :rarm :inverse-kinematics pose :debug-view :no-message))
      (send *robot* :reset-manip-pose)
      (send *robot* :fix-leg-to-coords foot-coords)
      (setq ik-result (fullbody-ik pose))
      (if ik-result (push ik-result *ik-result*))
      ;; (if ik-result
      ;;     (return-from :solve-ik nil)
      ;;   )
      )
    )
  )


(defun init-robot ()
  (hrp2jsknts-init)
  (setq *robot* *hrp2jsknts*)
  (unless (boundp '*tfl*)
    (setq *tfl* (instance ros::transform-listener :init)))
  (setq *hand-coords*  (send (send (send *robot* :hand :rarm) :copy-worldcoords) :transformation (send (send *robot* :rarm :end-coords) :copy-worldcoords)))
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
(defun go-pos-exec ()
  (let ((trans (send *robot* :foot-midcoords)))
    (let ((x (* 0.001 (elt (send trans :worldpos) 0)))
          (y (* 0.001 (elt (send trans :worldpos) 1)))
          (theta (rad2deg (elt (car (send trans :rpy-angle)) 0))))
      (go-pos x y theta)
      )
    ))

(defun go-pos (x y theta)
  (call-empty "/start_record")
  (send *ri* :go-pos x y theta)
  (unix::sleep 1)
  (call-empty "/stop_record")
  )

(defun go-pos-coords (coords)
  (setq coords2 (send (send coords :copy-worldcoords) :translate (float-vector 0 40 -160)))
  (send *robot* :fix-leg-to-coords (send (make-coords :pos (float-vector (- (elt (send coords2 :pos) 0) 450) 0 0)) :rotate (deg2rad 0) :z))
  )
(defun fullbody-move-end-pos (arm x y z)
  (fullbody-ik (send (send (send *robot* arm :end-coords) :copy-worldcoords) :translate (float-vector x y z)))
  )
(defun get-trans (msg)
  (send *tfl* :lookup-transform
        "/ground"
        (send msg :header :frame_id)
        (ros::time 0))
  )

(defun callback-common (msg2 arm &key (initialize? nil))
  (setq msg (send msg2 :pose_array))
  (setq trans-ground->kinfu
        (get-trans msg))
  ;; (ros::service-call "/file_getter" (instance std_srvs::Emptyrequest :init))
  (setq *wgrs* (set-wgrs msg))
  (if initialize?
    (progn
      (load "/home/leus/.ros/tmp_model.l")
      (tmp-world)
      (send *tmp-world* :newcoords (send trans-ground->kinfu :copy-worldcoords))
      (objects (list *robot* *tmp-world*))
      (reset-manip-pose-with-bend)
      (go-pos-coords (elt lst 0))
      )
    )
  (setq *ret* (solve-rrt *wgrs* arm))
  ;; (send *robot* :fix-leg-to-coords (make-coords))
  ;; (send *robot* :fix-leg-to-coords (make-coords :pos (float-vector 200 200 0)))  
  )

(defun callback2 (msg)
  (callback-common msg :rarm :initialize? t)
  )

(defun callback3 (msg)
  (callback-common msg :larm :initialize? nil)
  )

(defun solve-rrt (wgrs rarm)
  (setq *solver*
        (instance rrt-jt-multigoal :init
                  *robot*
                  :rarm
                  wgrs
                  :stop 2000
                  ;; :end-coords *hand-new-endcoords*
                  :end-coords (send *robot* :rarm :end-coords)
                  ))
  ;; (setf (send *robot* :rarm :end-coords) *hand-new-endcoords*)
  (send *solver* :solve)
  )

(defun set-wgrs (msg)
  (setq lst nil)
  (let (wgrs)
    (setq wgrs nil)
    (dolist (obj (send msg :poses))
      (let ((trs
             (send (send trans-ground->kinfu :copy-worldcoords)
                   :transform (ros::tf-pose->coords  obj))))
        (send trs :translate (float-vector 0 -50 50)) ;; offset for avoid collision
      (send trs :transform (send *hand-coords* :copy-worldcoords))
      ;; (send trs :rotate (deg2rad -90) :y)
      ;; (send trs :rotate (deg2rad -180) :y)
      (let
          ((trs-0 (send trs :copy-worldcoords)))
        (push trs-0 lst)
        (push
         (instance workspace-goal-region :init trs-0 :min-range (float-vector 0 0 0 0 0 0) :max-range (float-vector 0 0 0 0 0 0))
         wgrs)
        )
      ))
    wgrs
    )
  )

(defun show-motion
  ()
  (dolist (state (send-all (send *ret* :solution-nodes) :state))
    (send *robot* :rarm :angle-vector state)
    (send *irtviewer* :draw-objects)
    (read-line))
  )
(defun show-grasp
  ()
  (dotimes (i (length lst)) (send (elt lst i) :draw-on :flush t :color (float-vector 1 0 0)))
  )

(defun reset-manip-pose-with-bend
  (&key (offset 0))
  (send *robot* :reset-manip-pose)
  (send *robot* :legs :move-end-pos (float-vector 0 0 offset))
  (send *robot* :head :neck-p :joint-angle 30)
  )

(defun do-motion
  ()
  (open-hand)
  (go-pos-exec)
  (reset-manip-pose-with-bend)
  (send *ri* :angle-vector (send *robot* :angle-vector) 10000)
  (send *ri* :wait-interpolation)
  (dolist (state (send-all (send *ret* :solution-nodes) :state))
    (send *robot* :rarm :angle-vector state)
    (send *irtviewer* :draw-objects)
    (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    )
  ;; offset
  (send *robot* :rarm :move-end-pos (float-vector 50 50 0))
  (send *robot* :look-at-target (send *robot* :rarm :end-coords))
  (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  (close-hand)
  )

(defun try-move
  (angle-vector)
  (setq *tmp* (send *ri* :state :reference-vector))
  (send *ri* :angle-vector angle-vector 20000)
  (dotimes (i 22) ;; 20 + 2 for avoid miss
    (unix::usleep 1000000)
    (call-image-save)
    )
  ;; save bmf file
  (save-bmf-file)
  ;; call moseg
  (call-moseg)
  )
(defun call-empty
  (name)
  (let ((req (instance std_srvs::EmptyRequest :init)) res)
    (setq res (ros::service-call name req))
    )
  )
(defun call-image-save
  ()
  (call-empty "/save_image_for_moseg/save")
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

(defun bend-legs ()
  (reset-manip-pose-with-bend :offset 150)
  (send *ri* :angle-vector (send *robot* :angle-vector) 10000)
  (send *ri* :wait-interpolation)
  (reset-manip-pose-with-bend :offset 0)
  (send *ri* :angle-vector (send *robot* :angle-vector) 10000)
  (send *ri* :wait-interpolation)
  )

(defun call-moseg
  ()
  (unix::system "cp ~/.ros/left00*.jpg $(rospack find openrave_test)/moseg/TrainingSet/Data/fromrobot/")
  (unix::system "mogrify -format ppm $(rospack find openrave_test)/moseg/TrainingSet/Data/fromrobot/left*")
  (cd "/home/leus/ros/indigo/src/openrave_test/moseg") ;; need to change dir
  (unix::system "rm -rf $(rospack find openrave_test)/moseg/TrainingSet/Data/fromrobot/FlowFiles")
  (unix::system "$(rospack find openrave_test)/moseg/MoSeg $(rospack find openrave_test)/moseg/filestructureTrainingSet.cfg fromrobot 0 20 4 20")
  (unix::system "./dens100gpu filestructureTrainingSetDensify.cfg fromrobot/image.ppm OchsBroxMalik4_all_0000020.00/fromrobot/Tracks20.dat -1 OchsBroxMalik4_all_0000020.00/fromrobot/DenseSegmentation")
  )

(ros::roseus "sample-grasp-node")
(init-robot)
(ros::subscribe "/grasp_finder_right/rave_grasp_result" openrave_test::RaveGraspArray #'callback)
(ros::subscribe "/second_grasp_array" openrave_test::RaveGraspArray #'callback3)
