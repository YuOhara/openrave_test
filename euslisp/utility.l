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
  (tc mt ll &key (draw? t) (translation-axis (list t t t)) (rotation-axis (list t t t)) (stop 40))
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
