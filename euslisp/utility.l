(defun fullbody-ik2
  (tc-hand)
  (let (tc)
    (setq tc
          (list
           tc-hand
           (send *robot* :rarm :end-coords :copy-worldcoords)
           (send *robot* :rleg :end-coords :copy-worldcoords)
           (send *robot* :lleg :end-coords :copy-worldcoords)))
    (with-move-target-link-list
     (mt ll *robot* (list :larm :rarm :rleg :lleg))
     (fullbody-inverse-kinematics-for-hrp2 tc mt ll :draw? t :translation-axis (list t t t t) :rotation-axis (list t t t t))
     )
    )
  )

(defun fullbody-ik
  (tc-hand &key (end-coords nil))
  (let (tc)
    (setq tc
          (list
           tc-hand
           (send *robot* :rleg :end-coords :copy-worldcoords)
           (send *robot* :lleg :end-coords :copy-worldcoords)))
    (with-move-target-link-list
     (mt ll *robot* (list :rarm :rleg :lleg))
     (if end-coords
         (setf (elt mt 0) end-coords)
         )
     (fullbody-inverse-kinematics-for-hrp2 tc mt ll :draw? nil)
     )
    )
  )

(defun sort-angle-vector 
  (&key (current nil))
  (let ((appended-list nil)
        (sorted-list nil)
        )
    (if current
        (setq *robot-angle* current)
      (setq *robot-angle* (send *ri* :state :potentio-vector))
      )
    (dotimes (i (length *ik-result*))
      (push (list (elt *ik-result* i) (elt *hand-result* i)) appended-list)
      )
    (ros::ros-info "before sort")
    (defun sort-func
      (vec1 vec2)
      (> (norm (v- (elt vec1 0) *robot-angle*)) (norm (v- (elt vec2 0) *robot-angle*)))
      )
    (ros::ros-info "midlle sort")
    (sort appended-list `sort-func)
    (ros::ros-info "after sort")
    (setq sorted-list appended-list)
    (setq *ik-result* nil)
    (setq *hand-result* nil)
    (dotimes (i (length sorted-list))
      (push (elt (elt sorted-list i) 0) *ik-result*);
      (push (elt (elt sorted-list i) 1) *hand-result*);
      )
    )
  )


(defun fullbody-inverse-kinematics-for-hrp2
  (tc mt ll &key (draw? nil) (translation-axis (list t t t)) (rotation-axis (list t t t)) (stop 30))
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
