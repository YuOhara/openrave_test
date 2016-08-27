(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.k")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus-add-srvs "std_srvs")

(defun init-robot ()
  (hrp2jsknts-init)
  (setq *robot* *hrp2jsknts*)
  (unless (boundp '*tfl*)
    (setq *tfl* (instance ros::transform-listener :init)))
  (setq *hand-coords*  (send (send (send (send *robot* :hand :rarm) :copy-worldcoords) :translate (float-vector 0 40 -150)) :transformation (send (send *robot* :rarm :end-coords) :copy-worldcoords)))
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
  ;; (load "/home/leus/.ros/tmp_model.l")
  ;; (tmp-world)
  ;; (send *tmp-world* :newcoords (send trans-ground->kinfu :copy-worldcoords))
  ;; (objects (list *robot* *tmp-world*))
  (setq lst nil)
  (let (ik-result)
    (dolist (obj (send msg :poses))
      (let ((trs
             (send (send trans-ground->kinfu :copy-worldcoords)
                   :transform (ros::tf-pose->coords  obj))))
        (push trs lst)
        ))
    (setq *ik-result* nil)
    (block :solve-ik
      (dolist (pose lst)
        (let ((pose-trans (send pose :transform (send *hand-coords* :copy-worldcoords))))
          ;; (setq ik-result (send *robot* :rarm :inverse-kinematics pose-trans :debug-view :no-message))
          (setq ik-result (fullbody-ik pose-trans))
          (if ik-result (push ik-result *ik-result*))
          ;; (if ik-result
          ;;     (return-from :solve-ik nil)
          ;;   )
          )
        )
      )
    ik-result
    )
  )

(setq *right-obstacle* :unknown)
(setq *left-obstacle* :unknown)

(defun indice-callback
  (msg)
  (setq *right-obstacle* :unknown)
  (setq *left-obstacle* :unknown)
  (if (equal (length (send (elt (send msg :cluster_indices) 1) :indices)) 0)
      (setq *right-obstacle* nil)
    (setq *right-obstacle* :clouded)
    )
  (if (equal (length (send (elt (send msg :cluster_indices) 0) :indices)) 0)
      (setq *left-obstacle* nil)
    (setq *left-obstacle* :clouded)
    )
  )
(defun renew-obstacle
  ()
  (setq *right-obstacle* :unknown)
  (setq *left-obstacle* :unknown)
  (ros::spin-once)
  )

(defun move-right
  ()
  (renew-obstacle)
  (if (equal *right-obstacle* nil)
      (progn 
        (send *ri* :go-pos 0.155 -0.269 15)
        :go-success
        )
    )
  nil
  )

(defun move-left
  ()
  (renew-obstacle)
  (if (equal *right-obstacle* nil)
      (send *ri* :go-pos 0.155 0.269 -15)
    :go-success
    )
  nil
  )

(defun move-head
  ()
  (send *robot* :angle-vector (send *ri* :state :reference-vector))
  (send *robot* :head :neck-y :joint-angle 20)
  (send *ri* :angle-vector (send *robot* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  (send *robot* :head :neck-y :joint-angle -20)
  (send *ri* :angle-vector (send *robot* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  (send *robot* :head :neck-y :joint-angle 0)
  (send *ri* :angle-vector (send *robot* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  )

(defun move-head-down
  ()
  (send *robot* :angle-vector (send *ri* :state :reference-vector))
  (send *robot* :head :neck-p :joint-angle 40)
  (send *ri* :angle-vector (send *robot* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  )

(defun fullbody-ik
  (tc-hand)
  (send *robot* :reset-manip-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (let (tc)
    (setq tc
          (list
           tc-hand
           (send *robot* :rleg :end-coords :copy-worldcoords)
           (send *robot* :lleg :end-coords :copy-worldcoords)))
    (with-move-target-link-list
     (mt ll *robot* (list :rarm :rleg :lleg))
     (fullbody-inverse-kinematics-for-hrp2 tc mt ll :draw? t)
     )
    )
  )

(defun fullbody-inverse-kinematics-for-hrp2
  (tc mt ll &key (draw? t) (translation-axis (list t t t)) (rotation-axis (list t t t)) (stop 50))
  (send *robot*
        :fullbody-inverse-kinematics tc
        :move-target mt :link-list ll
        :additional-weight-list
        (list (list (send *robot* :rleg :toe-p :child-link) 0)
              (list (send *robot* :lleg :toe-p :child-link) 0)
              (list (send *robot* :torso :waist-p :child-link) 0.1)
              (list (send *robot* :torso :waist-y :child-link) 0.2)
              )
        :root-link-virtual-joint-weight (float-vector 0.1 0.1 0.1 0.1 0.5 0.5)
        :target-centroid-pos (apply #'midpoint 0.5 (send *robot* :legs :end-coords :worldpos))
        :translation-axis translation-axis
        :rotation-axis rotation-axis
        :max (float-vector  500  500  0  20  20  10) ;; for hrp2
        :min (float-vector  -500  -500  -200  -20  -20  -10) ;; for hrp2
        :stop stop
        :debug-view (if draw? :no-message nil)
        )
  )

(defun hand-back
  ()
  (send *robot* :rarm :move-end-pos (float-vector -60 -60 0))
  )

(defun move-sample
  ()
  (send *ri* :go-pos 0 0.2 0)
  (send *ri* :go-pos 0 -0.4 0)
  (send *ri* :go-pos 0 0.2 0)
  )

(defun open-foot
  ()
  (let* ((lleg-coords-orig (send (send *robot* :lleg :end-coords) :copy-worldcoords))
         (rleg-coords-orig (send (send *robot* :rleg :end-coords) :copy-worldcoords))
         lleg-coords-new rleg-coords-new
         footstep-list
         )
    (send *robot* :lleg :move-end-pos #f(0 50 0) :world)
    (send *robot* :rleg :move-end-pos #f(0 0 0) :world)
    (send *irtviewer* :draw-objects)
    (setq lleg-coords-new (send (send *robot* :lleg :end-coords) :copy-worldcoords))
    (setq rleg-coords-new (send (send *robot* :rleg :end-coords) :copy-worldcoords))
    (send lleg-coords-orig :name :lleg)
    (send rleg-coords-orig :name :rleg)
    (send lleg-coords-new :name :lleg)
    (send rleg-coords-new :name :rleg)
    (setq footstep-list (list rleg-coords-orig lleg-coords-new))
    (send *ri* :set-foot-steps footstep-list)
    )
  )


(defun left-foot
  ()
  (send *ri* :set-auto-balancer-param :default-zmp-offsets (list (float-vector 0 50 0) (float-vector 0 50 0) (float-vector 0 0 0) (float-vector 0 0 0)))
  )

(defun middle-foot
  ()
  (send *ri* :set-auto-balancer-param :default-zmp-offsets (list (float-vector 0 0 0) (float-vector 0 0 0) (float-vector 0 0 0) (float-vector 0 0 0)))
  )

(defun right-foot
  ()
  (send *ri* :set-auto-balancer-param :default-zmp-offsets (list (float-vector 0 -50 0) (float-vector 0 -50 0) (float-vector 0 0 0) (float-vector 0 0 0)))
  )

(ros::roseus "sample-grasp-node")
(init-robot)
(ros::subscribe "/grasp_caluculation_result" geometry_msgs::PoseArray #'callback)
(ros::subscribe "/attention_clipper/output/cluster_point_indices" jsk_recognition_msgs::ClusterPointIndices #'indice-callback)
