(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
(load "~/prog/euslib/jsk/searchlib.l")
(load "~/prog/euslib/demo/ohara/motion_plan/euslisp/rrt-jt-multigoal.l")
(load "~/prog/euslib/demo/ohara/motion_plan/euslisp/util.l")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-srvs "std_srvs")
(ros::roseus-add-srvs "openrave_test")
(ros::roseus-add-msgs "openrave_test")
(ros::roseus-add-msgs "jsk_recognition_msgs")
(load "package://openrave_test/euslisp/utility.l")

(defmethod rrt-jt-multigoal
  (:collision-free ;; need to implement this
   ()
   (not (check-collision :obj *tmp-world*
                         ;; :link-list (send *robot* arm :links)
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
    (send *robot* arm :angle-vector (send (elt (send s :solution-nodes) (- (length (send s :solution-nodes)) 1)) :state))
    (if
        (send *robot* arm :move-end-pos (float-vector 50 (if (equal arm :rarm) 50 -50) 0))
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

(defun show-pose (arm i)
  (let ((foot-coords (send *robot* :foot-midcoords)))
    (progn (send *robot* :angle-vector (elt *ik-result* i)) (send *robot* :fix-leg-to-coords foot-coords))
    (show-hand arm i)
    (look-at-arm arm)
    (send *irtviewer* :draw-objects)
    )
  )
(defun look-at-arm (arm)
  (send *robot* :look-at-target (send (send *robot* arm :end-coords :copy-worldcoords) :translate (float-vector 100 (if (equal arm :rarm) 50 -50) 0)))
  )
(defun show-hand (arm i)
  (send *robot* :hand arm :angle-vector 
        (v+ (elt *hand-result* i)
            (float-vector 0 -20 10 0 10 20)))
  )
(defun all-ik ()
  (setq *ik-result* nil)
  (setq *hand-result* nil)
  (let ((foot-coords (send *robot* :foot-midcoords)))
    (dotimes (i (length lst))
      ;; (setq ik-result (send *robot* :rarm :inverse-kinematics pose :debug-view :no-message))
      (setq ik-result (fullbody-ik (elt lst i)))
      (if (and ik-result (send *robot* :rarm :move-end-pos (float-vector 50 50 0)))
          (progn
            (push ik-result *ik-result*)
            (push (coerce (mapcar #'rad2deg (coerce (send (elt *finger* i) :data) cons)) float-vector) *hand-result*)
            )
        )
      (send *robot* :reset-manip-pose)
      (send *robot* :fix-leg-to-coords foot-coords)
      ;; (if ik-result
      ;;     (return-from :solve-ik nil)
      ;;   )
      )
    )
  )

(defun all-ik2 ()
  (setq *ik-result* nil)
  (setq *hand-result* nil)
  (let ((foot-coords (send *robot* :foot-midcoords))
        (angle-vec (send *robot* :angle-vector)))
    (dotimes (i (length lst))
      ;; (setq ik-result (send *robot* :rarm :inverse-kinematics pose :debug-view :no-message))
      (setq ik-result (fullbody-ik2 (elt lst i)))
      (if (and ik-result (send *robot* :larm :move-end-pos (float-vector 50 -50 0)) (send *robot* :arms :move-end-pos (float-vector 0 0 100) :world))
          (progn
            (push ik-result *ik-result*)
            (push (coerce (mapcar #'rad2deg (coerce (send (elt *finger* i) :data) cons)) float-vector) *hand-result*)
            )
        )
      (send *robot* :angle-vector angle-vec)
      (send *robot* :fix-leg-to-coords foot-coords)
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
  (setq *hand-coords-r*  (send (send (send *robot* :hand :rarm) :copy-worldcoords) :transformation (send (send *robot* :rarm :end-coords) :copy-worldcoords)))
  (setq *hand-coords-l*  (send (send (send *robot* :hand :larm) :copy-worldcoords) :transformation (send (send *robot* :larm :end-coords) :copy-worldcoords)))
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
      (go-pos2 x y theta)
      )
    ))

(defun coords->xytheta (trans)
  (let ((x (* 0.001 (elt (send trans :worldpos) 0)))
        (y (* 0.001 (elt (send trans :worldpos) 1)))
        (theta (rad2deg (elt (car (send trans :rpy-angle)) 0))))
    (list x y theta)
    )
  )

(defun xytheta->coords (x y theta)
  (let ((coords (make-coords :pos (float-vector (* x 1000) (* y 1000) 0))))
    (send coords :rotate (deg2rad  theta) :z)
    coords
    )
  )

(defun go-pos (x y theta)
  (call-empty "/start_record")
  (send *ri* :go-pos x y theta)
  (unix::sleep 2)
  (call-empty "/stop_record")
  )

(defun go-pos2 (x y theta)
  (let (coords-beforehoge coords-afterhoge)
    (setq coords-before
          (send *tfl* :lookup-transform "/triger_base_map" "ground" (ros::time 0)))
    (go-pos x y theta)
    (setq coords-after
          (send *tfl* :lookup-transform "/triger_base_map" "ground"  (ros::time 0)))
    (let* ((trans-true (send coords-before :transformation coords-after))
          (trans-needed (xytheta->coords x y theta))
          (trans (send trans-true :transformation trans-needed)))
      (let ((x2 (* 0.001 (elt (send trans :worldpos) 0)))
            (y2 (* 0.001 (elt (send trans :worldpos) 1)))
            (theta2 (rad2deg (elt (car (send trans :rpy-angle)) 0))))
        (ros::ros-info (format nil "go-pos ~a ~a ~a" x2 y2 theta2))
        (go-pos x2 y2 theta2)
        )
      )
    )
  )

(defun go-pos-coords (coords)
  (setq coords2 (send (send coords :copy-worldcoords) :translate (float-vector 0 40 -160)))
  (send *robot* :fix-leg-to-coords (send (make-coords :pos (float-vector (- (elt (send coords2 :pos) 0) 450) 0 0)) :rotate (deg2rad 0) :z))
  )
(defun fullbody-move-end-pos (arm x y z &key (world nil))
  (fullbody-ik (send (send (send *robot* arm :end-coords) :copy-worldcoords) :translate (float-vector x y z) (if world :world :local)))
  )
(defun get-trans (msg)
  (send *tfl* :lookup-transform
        "/ground"
        (send msg :header :frame_id)
        (ros::time 0))
  )

(defun callback-common (msg2 arm &key (initialize? nil))
  (setq msg (send msg2 :pose_array))
  (setq *finger* (send msg2 :grasp_array))
  (setq trans-ground->kinfu
        (get-trans msg))
  ;; (ros::service-call "/file_getter" (instance std_srvs::Emptyrequest :init))
  (setq *wgrs* (set-wgrs msg arm))
  (if initialize?
    (progn
      (load "~/.ros/tmp_model.l")
      (tmp-world)
      (send *tmp-world* :newcoords (send trans-ground->kinfu :copy-worldcoords))
      (objects (list *robot* *tmp-world*))
      (reset-manip-pose-with-bend)
      ;; (go-pos-coords (elt lst 0))
      (send *robot* :fix-leg-to-coords (make-coords :pos (float-vector 100 0 0)))
      )
    )
  ;; (setq *ret* (solve-rrt *wgrs* arm))
  ;; (send *robot* :hand arm :angle-vector (v+ (return-finger-param arm) (float-vector 0 -10 10 0 10 10)))
  (if (equal arm :rarm) (all-ik) 
    (progn
      (send *robot* :angle-vector *tmp*)
      (send *robot* :fix-leg-to-coords (make-coords :pos (float-vector 100 0 0)))
      (all-ik2)
      )
    )
  (sort-angle-vector)
  (show-pose arm 0)
  ;; (send *robot* :fix-leg-to-coords (make-coords))
  ;; (send *robot* :fix-leg-to-coords (make-coords :pos (float-vector 200 200 0)))  
  )

(defun callback2 (msg)
  (callback-common msg :rarm :initialize? t)
  )

(defun callback3 (msg)
  (setq *msg-l* msg)
  (callback4 msg)
  )

(defun callback4 (msg)
  (callback-common msg :larm :initialize? nil)
  )

(defun solve-rrt (wgrs arm)
  (setq *solver*
        (instance rrt-jt-multigoal :init
                  *robot*
                  arm
                  wgrs
                  :stop 2000
                  :end-coords (send *robot* arm :end-coords)
                  ))

  (send *solver* :solve)
  )

(defun return-finger-param (arm)
  (let (
        (answer nil)
        (min-dist 10000)
        (state (send (elt (send *ret* :solution-nodes) (- (length (send *ret* :solution-nodes)) 1)) :state))
        )
      (send *robot* arm :angle-vector state)
      (send *robot* arm :move-end-pos (float-vector 50 (if (equal arm :rarm) 50 -50) 0))
      (dotimes (i (length lst))
        (let ((dist-tmp (norm (v- (send (elt lst i) :worldpos) (send (send *robot* arm :end-coords :copy-worldcoords) :worldpos)))))
          (if (< dist-tmp min-dist)
              (setq min-dist dist-tmp)
            (setq answer i)
            )
          )
        )
      (coerce (mapcar #'rad2deg (coerce (send (elt *finger* answer) :data) cons)) float-vector)
      )
  )

(defun set-wgrs (msg arm)
  (setq lst nil)
  (let (wgrs)
    (setq wgrs nil)
    (dolist (obj (send msg :poses))
      (let ((trs
             (send (send trans-ground->kinfu :copy-worldcoords)
                   :transform (ros::tf-pose->coords  obj))))
        (send trs :transform (send (if (equal arm :rarm ) *hand-coords-r* *hand-coords-l*) :copy-worldcoords))
        (send trs :translate (float-vector -50 (if (equal arm :rarm) -50 50) 0)) ;; offset for avoid collision
        (setq trs (send (send (send *robot* :foot-midcoords) :copy-worldcoords) :transform trs))
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
  (arm)
  (dolist (state (send-all (send *ret* :solution-nodes) :state))
    (send *robot* arm :angle-vector state)
    (send *irtviewer* :draw-objects)
    (if (send *robot* :self-collision-check)
        (ros::ros-info "self collision!"))
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
  (arm)
  (open-hand arm)
  (go-pos-exec)
  (reset-manip-pose-with-bend)
  (send *ri* :angle-vector (send *robot* :angle-vector) 10000)
  (send *ri* :wait-interpolation)
  (dolist (state (send-all (send *ret* :solution-nodes) :state))
    (send *robot* arm :angle-vector state)
    (send *irtviewer* :draw-objects)
    (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    )
  ;; offset
  (send *robot* arm :move-end-pos (float-vector 50 (if (equal arm :rarm) 50 -50) 0))
  (look-at-arm arm)
  (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  (close-hand)
  )

(defun do-motion2 (arm)
  (if (equal arm :rarm) (go-pos-exec))
  (send *ri* :angle-vector (send *robot* :angle-vector) 20000)
  (send *ri* :wait-interpolation)
  (if (send *robot* arm :move-end-pos (float-vector 50 (if (equal arm :rarm) 50 -50) 0 ))
      (progn
        (look-at-arm arm)
        (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
        (send *ri* :wait-interpolation)
        )
    (return)
    )
  (close-hand-more arm)
  (if (equal arm :rarm)
      (progn
        (try-grasp arm)
        (if
            (fullbody-move-end-pos :rarm 0 0 70 :world t)
            ;(send *robot* arm :move-end-pos (float-vector 0 0 70) :world)
            (progn
              (if (y-or-n-p "try move?")
                  (try-move (send *robot* :angle-vector))
                (return)
                )
              (if (y-or-n-p "move back?")
                  (progn
                    (send *ri* :angle-vector *tmp* 10000)
                    (send *ri* :wait-interpolation))
                (return)
                )
              )
          (progn
            (ros::ros-info "ik failed, please implement fullbody-ik(todo)")
            (return)
            )
          )
        )
    (progn
      (hand-model2real)
      )
    )
  )

(defun try-grasp
  (arm)
  (unix::system "rosnode kill /save_image_for_moseg") ;; sometimes no res? also
  (let (
        (dist-angle (send *robot* :hand arm :angle-vector))
        (tmp-angle (if (equal arm :rarm)
                   (subseq (send (send *ri* :hand-state) :reference) 0 6)
                 (subseq (send (send *ri* :hand-state) :reference) 6 12)
                 ))
        (devide-num 10))
    (dotimes (i devide-num)
      (send *robot* :hand arm :angle-vector (v+ (scale (/ (+ i 1.0) devide-num) dist-angle) (scale (- 1 (/ (+ i 1.0) devide-num)) tmp-angle)))
      (ros::ros-info (format nil "~a" (send *robot* :hand :rarm :angle-vector)))
      (hand-model2real)
      (unix::usleep 1000000)
      (call-image-save)
      )
    )
  (call-image-save) ;; hoken
  (call-moseg :start 0 :stop 10 :direc "fromrobot2")
  )

(defun try-move
  (angle-vector)
  (unix::system "rosnode kill /save_image_for_moseg") ;; sometimes no res? also need to reset index of image_savere index
  (unix::usleep 1000000)
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
  (arm)
  (send *robot* :hand arm :angle-vector (float-vector 90.0 90.0 0.0 0.0 -40.0 -40.0))
  (hand-model2real)
  )

(setq hand-angle-list
      (list
       #f(90.0 20.0 0.0 10.0 60.0 60.0) ;; both
       #f(90.0 -30.0 0.0 10.0 -20.0 -20.0) ;; thumb
       #f(90.0 90.0 0.0 0.0 110.0 110.0) ;; middle
       ))

(defun close-hand-more
  (arm)
  (let ((min-angle nil)
        (min-diff 10000)
        (current (send *robot* :hand arm :angle-vector)))
    (dolist (h-angle hand-angle-list)
      (if (< (norm (v- current h-angle)) min-diff)
          (progn
            (setq min-angle h-angle)
            (setq min-diff (norm (v- current h-angle)))
            )
          )
      )
    (if min-angle
        (send *robot* :hand arm :angle-vector min-angle)
      (ros::ros-info "hand angle empty")
      )
    )
                                        ; (hand-model2real)
  )

(defun demo ()
  (open-hand :rarm)
  (open-hand :larm)
  (if (y-or-n-p "look-around?")
      (look-around1)
    (return)
    )
  (if (y-or-n-p "done grasp caluculation?")
      (ros::spin-once)
    (return)
    )
  (if (y-or-n-p "first grasp starts, ok?")
      (do-motion2 :rarm)
    (return)
    )
  (if (y-or-n-p "done second grasp caluculation?")
      (ros::spin-once)
    (return)
    )
  (if (y-or-n-p "second grasp starts, ok?")
      (do-motion2 :larm)
    (return)
    )
  (if (y-or-n-p "move up?")
      (if (send *robot* :arms :move-end-pos (float-vector 0 0 100) :world)
          (progn
            (send *ri* :angle-vector (send *robot* :angle-vector) 10000)
            (send *ri* :wait-interpolation)
            )
        )
    )
  (ros::ros-info "done all demo!")
  )

(defun close-hand-common
  (arm index)
  (send *robot* :hand arm :angle-vector (elt hand-angle-list index))
  (hand-model2real)
  )

(defun close-hand0
  (arm)
  (close-hand-common arm 0)
  )

(defun close-hand1
  (arm)
  (close-hand-common arm 1)
  )

(defun close-hand2
  (arm)
  (close-hand-common arm 2)
  )


(defun full-close-hand
  (arm)
  (send *robot* :hand arm :r_thumbcm_p :joint-angle -30)
  (hand-model2real)
  (send *robot* :hand arm :r_middlepip_r :joint-angle 110)
  (hand-model2real)
  (send *robot* :hand arm :r_indexmp_r :joint-angle 30)
  (hand-model2real)
  (send *robot* :hand arm :r_indexpip_r :joint-angle 60)
  (hand-model2real)
  )



(defun hand-model2real (&key (time 500) (wait? t))
  (send *ri* :hand-angle-vector (apply #'concatenate float-vector (send *robot* :hand :arms :angle-vector)) time)
  (when wait? (send *ri* :hand-wait-interpolation))
  )

(defun bend-legs ()
  (reset-manip-pose-with-bend :offset 150)
  (send *ri* :angle-vector (send *robot* :angle-vector) 10000)
  (send *ri* :wait-interpolation)
  (reset-manip-pose-with-bend :offset 0)
  (send *ri* :angle-vector (send *robot* :angle-vector) 10000)
  (send *ri* :wait-interpolation)
  )

(defun call-moseg
  (&key (start 0) (stop 20) (direc "fromrobot"))
  (unix::system (format nil "cp ~~/.ros/left00*.jpg $(rospack find openrave_test)/moseg/TrainingSet/Data/~a/" direc))
  (unix::system (format nil "mogrify -format ppm $(rospack find openrave_test)/moseg/TrainingSet/Data/~a/left*" direc))
  (unix::system (format nil "rm -rf $(rospack find openrave_test)/moseg/TrainingSet/Data/~a/FlowFiles" direc))
  (unix::system (format nil "cd $(rospack find openrave_test)/moseg; $(rospack find openrave_test)/moseg/MoSeg $(rospack find openrave_test)/moseg/filestructureTrainingSet.cfg ~a ~a ~a 4 20" direc start stop))
  (unix::system (format nil "cd $(rospack find openrave_test)/moseg; ./dens100gpu filestructureTrainingSetDensify.cfg ~a/image.ppm OchsBroxMalik4_all_0000020.00/~a/Tracks~a.dat -1 OchsBroxMalik4_all_0000020.00/~a/DenseSegmentation" direc direc stop direc))
  )


(defun look-around1 ()
  (progn (bend-legs) (go-pos 0 0.2 0) (bend-legs) (go-pos 0 -0.4 0) (bend-legs) (go-pos 0 0.2 0))
  )

(ros::roseus "sample-grasp-node")
(init-robot)
(ros::subscribe "/grasp_finder_right/rave_grasp_result" openrave_test::RaveGraspArray #'callback)
(ros::subscribe "/second_rave_grasp_array" openrave_test::RaveGraspArray #'callback3)

