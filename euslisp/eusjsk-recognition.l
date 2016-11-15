(ros::roseus-add-msgs "jsk_recognition_msgs")

(defun ros::bounding-box->cube (bbox &optional (base-frame-id nil))
  ;; bbox is framed-msg, so applyed tf
  (let* ((dims (ros::tf-point->pos (send bbox :dimensions)))
         (bx (make-cube (elt dims 0) (elt dims 1) (elt dims 2)))
         (bf (or base-frame-id (send bbox :header :frame_id))))
    (unless (boundp '*tfl*)
      (setq *tfl* (instance ros::transform-listener :init)))
    (send bx :newcoords
          (send (send *tfl* :lookup-transform bf (send bbox :header :frame_id) (ros::time 0)) :transform
                (ros::tf-pose->coords
                 (send bbox :pose))))
    bx))
