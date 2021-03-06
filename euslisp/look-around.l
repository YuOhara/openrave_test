(defun left-foot
  ()
  (send *ri* :set-auto-balancer-param :default-zmp-offsets (list (float-vector 0 80 0) (float-vector 0 80 0) (float-vector 0 0 0) (float-vector 0 0 0)))
  )

(defun middle-foot
  ()
  (send *ri* :set-auto-balancer-param :default-zmp-offsets (list (float-vector 0 0 0) (float-vector 0 0 0) (float-vector 0 0 0) (float-vector 0 0 0)))
  )

(defun right-foot
  ()
  (send *ri* :set-auto-balancer-param :default-zmp-offsets (list (float-vector 0 -80 0) (float-vector 0 -80 0) (float-vector 0 0 0) (float-vector 0 0 0)))
  )

(defun open-foot (offset)
  (send *robot* :reset-manip-pose)
  (let* ((lleg-coords-orig (send (send *robot* :lleg :end-coords) :copy-worldcoords))
         (rleg-coords-orig (send (send *robot* :rleg :end-coords) :copy-worldcoords))
         lleg-coords-new rleg-coords-new
         footstep-list
         )
    (send *robot* :lleg :move-end-pos (float-vector 0 offset 0) :world)
    (send *robot* :rleg :move-end-pos (float-vector 0 (- offset) 0) :world)
    ;; (send *irtviewer* :draw-objects)
    (setq lleg-coords-new (send (send *robot* :lleg :end-coords) :copy-worldcoords))
    (setq rleg-coords-new (send (send *robot* :rleg :end-coords) :copy-worldcoords))
    (send lleg-coords-orig :name :lleg)
    (send rleg-coords-orig :name :rleg)
    (send lleg-coords-new :name :lleg)
    (send rleg-coords-new :name :rleg)
    (setq footstep-list (list rleg-coords-orig lleg-coords-new rleg-coords-new))
    (send *ri* :set-foot-steps footstep-list)
    )
  )

(defun look-around2
  ()
  (send *robot* :reset-manip-pose)
  (send *robot* :head :neck-p :joint-angle 30)
  (send *ri* :angle-vector (send *robot* :angle-vector) 10000)
  (send *ri* :wait-interpolation)
  (send *ri* :set-auto-balancer-param :zmp-transition-time 2.0)
  (open-foot 40)
  (send *ri* :autobalancerservice_waitfootsteps)
  (right-foot)
  (send *robot* :legs :move-end-pos (float-vector 100 0 0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 10000)
  (middle-foot)
  (left-foot)
  (send *robot* :legs :move-end-pos (float-vector -100 0 0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 10000)
  (send *ri* :wait-interpolation)
  (middle-foot)
  (open-foot 0)
  (send *ri* :autobalancerservice_waitfootsteps)
  )
