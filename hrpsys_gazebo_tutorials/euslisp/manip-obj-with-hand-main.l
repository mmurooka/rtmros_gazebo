#!/usr/bin/env roseus

#-:jsk
(jsk)
#-:rbrain-basic
(rbrain)


(ros::roseus "manip_obj_with_hand")

(load "package://hrpsys_gazebo_tutorials/euslisp/gazebo-plugin-utils")
(load "package://hrpsys_gazebo_tutorials/euslisp/hand-command-publisher.l")
(load "package://hrpsys_gazebo_tutorials/euslisp/drum_body.l")
(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp3hand-utils.l")
(load "models/single-arrow-object.l")

(load "package://hrpsys_gazebo_tutorials/euslisp/manip-obj-with-hand-manip.l")
(load "package://hrpsys_gazebo_tutorials/euslisp/manip-obj-with-hand-generate.l")
(load "package://hrpsys_gazebo_tutorials/euslisp/manip-obj-with-hand-analyze.l")
(load "package://hrpsys_gazebo_tutorials/euslisp/manip-obj-with-hand-plot.l")
(load "package://hrpsys_gazebo_tutorials/euslisp/manip-obj-with-hand-strategy.l")



(defun usage ()
  (warn "(init) ;; initialize~%")
  (warn "(try) ;; try manip, save log, and plot graph~%")
  (warn "(try-once (car *manip-list*)) ;; play log manip~%")
  )

(defun init (&key (model-num 6) (model-name "cupboard_wh"))

  ;; load controller parameter for hand
  (unix::system "rosparam load `rospack find hrpsys_gazebo_tutorials`/config/HRP3HAND_L.yaml HRP3HAND_L")
  (unix::system "rosparam load `rospack find hrpsys_gazebo_tutorials`/config/HRP3HAND_R.yaml HRP3HAND_R")

  (setq *gplist* nil)
  (dotimes (i model-num)
    ;; add hand model to the gazebo world
    (unix::system (format nil "gzfactory spawn -f `rospack find hrpsys_gazebo_tutorials`/environment_models/~a/model~d.urdf -m ~a -x 0 -y 0 -z 1" model-name (+ i 1) model-name))

    ;; make gazebo plugin interface
    (eval (list 'setq (read-from-string (format nil "*gp~d*" (+ i 1))) (init-gazebo-plugin (format nil "obj~d" (+ i 1)))))
    (eval (list 'push-back (read-from-string (format nil "*gp~d*" (+ i 1))) '*gplist*))) 

  ;; make controller interface
  (setq *hc* (instance hand-command-publisher-class :init))
  (unix::usleep (* 1000 1000))

  ;; make and view euslisp model
  (setq *ar1* (single-arrow))
  (setq *ar2* (single-arrow))
  (setq *lhand* (hrp3hand_l))  
  (setq *rhand* (hrp3hand_r))  
  (setq *ground* (instance ground :init 5000 5000 450 450 "702-floor-tex.jpg" :name "702-ground"))
  (cond 
   ((equal model-name "cupboard_wh")
    (setq *obj* (make-73b2-cupboard-without-all-door))
    (send *lhand* :newcoords (send *obj* :handle :name :left-front-handle1))
    (send *rhand* :newcoords (send *obj* :handle :name :right-front-handle1)))
   ((equal model-name "drum_wh")
    (setq *obj* (instance drum_body_gkmodel0_nil_link_geom0 :init))
    (send *obj* :newcoords (make-coords))
    (send *lhand* :newcoords (make-coords :pos (v+ (send *obj* :worldpos) (float-vector 1800 -270 800)):rot (rpy-matrix pi -pi/2 0)))
    (send *rhand* :newcoords (make-coords :pos (v+ (send *obj* :worldpos) (float-vector 1800 410 800)):rot (rpy-matrix pi -pi/2 0)))
    ))
  (send *obj* :assoc *lhand*)
  (send *obj* :assoc *rhand*)
  (unless (boundp '*pickview*)
    (pickview :no-menu t))
  (objects (list *obj* *ground* *lhand* *rhand* *ar1* *ar2*))

  ;; set hook-pose
  (set-hand-pose :hook-pose :arms)
  (unix::usleep (* 1000 1000))

  ;; init graph
  (unless (boundp '*gnuplot-cont-st*)
    (setq *gnuplot-cont-st* (gnuplot)))
  (unless (boundp '*gnuplot-2d-coord*)
    (setq *gnuplot-2d-coord* (gnuplot)))

  ;; init variables
  (setq *manip-cnt* 0)
  (setq *manip-list* nil)
  (setq *cont-st-list* nil)
  (setq *2d-coord-list* nil)
  (setq *fv-list* nil)
  (setq *dt* 0.002)
  (send *obj* :put :vcont (list nil nil nil nil))
  (send *obj* :put :lcont (list nil nil nil nil))
  (send *obj* :put :fcont nil)
  (send *obj* :put :cont-st :face)
  (send *obj* :put :2d-pos (float-vector 0 0))
  (send *obj* :put :2d-angle 0)
  )

(defun init-drum ()
  (init :model-name "drum_wh"))
