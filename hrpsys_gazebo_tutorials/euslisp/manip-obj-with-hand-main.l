#!/usr/bin/env roseus

#-:jsk
(jsk)
#-:rbrain-basic
(rbrain)


(ros::roseus "manip_obj_with_hand")

(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp3hand-utils.l")
(load "models/single-arrow-object.l")

(load "package://hrpsys_gazebo_tutorials/euslisp/manip-obj-with-hand-generate.l")
(load "package://hrpsys_gazebo_tutorials/euslisp/manip-obj-with-hand-manip.l")
(load "package://hrpsys_gazebo_tutorials/euslisp/manip-obj-with-hand-analyze.l")
(load "package://hrpsys_gazebo_tutorials/euslisp/manip-obj-with-hand-plot.l")
(load "package://hrpsys_gazebo_tutorials/euslisp/manip-obj-with-hand-strategy.l")

(load "package://hrpsys_gazebo_tutorials/euslisp/hand-command-publisher.l")
(load "package://hrpsys_gazebo_tutorials/euslisp/drum_body.l")


(defun usage-main
  ()

  (warn ";; main ;;;;;;;;;;;;;;;;;~%")
  (warn "(init) ;; initialize~%")
  (warn "(main) ;; initialize~%")
  )

(defun usage
  ()

  (usage-main)
  (usage-generate)
  (usage-manip)
  (usage-analyze)
  (usage-plot)
  (usage-strategy)
  (usage-util)
  )

;; initialize ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun init
  (&key (model-num 3) (model-name "cupboard_wh") (force nil))

  (when (and (boundp '*manip-obj-with-hand-init-flag*) (not force))
    (init-variable)
    (return-from init nil))

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
  (setq *arlist* nil)
  (setq *handlist* nil)
  (setq *objlist* nil)
  (dotimes (i model-num)
    ;; make arrow
    (eval (list 'setq (read-from-string (format nil "*left-ar~d*" (+ i 1))) (single-arrow)))
    (eval (list 'setq (read-from-string (format nil "*right-ar~d*" (+ i 1))) (single-arrow)))
    (eval (list 'setq '*left-ar* (read-from-string (format nil "*left-ar~d*" (+ i 1)))))
    (eval (list 'setq '*right-ar* (read-from-string (format nil "*right-ar~d*" (+ i 1)))))
    (push-back *left-ar* *arlist*)
    (push-back *right-ar* *arlist*)
    ;; make hand
    (eval (list 'setq (read-from-string (format nil "*lhand~d*" (+ i 1))) (hrp3hand_l)))
    (eval (list 'setq (read-from-string (format nil "*rhand~d*" (+ i 1))) (hrp3hand_r)))
    (eval (list 'setq '*lhand* (read-from-string (format nil "*lhand~d*" (+ i 1)))))
    (eval (list 'setq '*rhand* (read-from-string (format nil "*rhand~d*" (+ i 1)))))
    (push-back *lhand* *handlist*)
    (push-back *rhand* *handlist*)
    ;; make obj
    (cond 
     ((equal model-name "cupboard_wh")
      (eval (list 'setq (read-from-string (format nil "*obj~d*" (+ i 1))) (make-73b2-cupboard-without-all-door)))
      (eval (list 'setq '*obj* (read-from-string (format nil "*obj~d*" (+ i 1)))))
      (send *lhand* :newcoords (send *obj* :handle :name :left-front-handle1))
      (send *rhand* :newcoords (send *obj* :handle :name :right-front-handle1))
      )
     ((equal model-name "drum_wh")
      (eval (list 'setq (read-from-string (format nil "*obj~d*" (+ i 1))) (instance drum_body_gkmodel0_nil_link_geom0 :init)))
      (eval (list 'setq '*obj* (read-from-string (format nil "*obj~d*" (+ i 1)))))
      (send *obj* :newcoords (make-coords))
      (send *lhand* :newcoords (make-coords :pos (v+ (send *obj* :worldpos) (float-vector 1800 -270 800)):rot (rpy-matrix pi -pi/2 0)))
      (send *rhand* :newcoords (make-coords :pos (v+ (send *obj* :worldpos) (float-vector 1800 410 800)):rot (rpy-matrix pi -pi/2 0)))
      ))
    ;; assoc hand
    (send *obj* :assoc *lhand*)
    (send *obj* :assoc *rhand*)
    (push-back *obj* *objlist*)
    )
  ;; make ground
  (setq *ground* (instance ground :init 5000 5000 450 450 "702-floor-tex.jpg" :name "702-ground"))
  ;; view models
  (unless (boundp '*pickview*)
    (pickview :no-menu t))
  (objects (append *arlist* *handlist* *objlist* (list *ground*)))

  ;; set hook-pose
  (set-hand-pose :hook-pose :arms)
  (unix::usleep (* 1000 1000))

  ;; init variables
  (init-variable)

  (setq *manip-obj-with-hand-init-flag* t)
  )

(defun init-cupboard ()
  (init :model-name "cupboard_wh"))

(defun init-drum ()
  (init :model-name "drum_wh"))


;; main function ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun main1
  ()

  ;; init
  (init)
  ;; loop
  (do-until-key
   (manip))
  ;; save
  (save-variable)
  ;; plot
  (plot-all)
  )

(defun main2
  (&optional (class-num 3) (no-analyze nil))

  ;; init
  (init)
  ;; load
  (load-variable)
  ;; analyze
  (unless (analyze :class-num class-num)
    (warn "failed classification~%")
    (return-from main2 nil))
  (dotimes (i (length (elt *all-class-data* 1)))
    ;; print
    (print-feature-value :class-num i)
    (print-class-info :class-num i)
    ;; plot
    (plot-all-class-data :class-num i :save nil)
    ))

(defun main3
  (&optional (class-num 3))

  ;; init
  (init)
  ;; load
  (load-variable)
  ;; analyze
  (unless (analyze :class-num class-num)
    (warn "failed classification~%")
    (return-from main3 nil))
  (dotimes (i (length (elt *all-class-data* 1)))
    ;; print
    (print-class-info :class-num i)
    ;; plot
    (plot-all-class-data :class-num i :save nil)
    ;; manip
    (manip-all-class-data :class-num i))
  )

(defun main-strategy
  ()

  ;; init
  (init)
  ;; loop
  (dolist (strategy *strategy-manip-list*)
    (manip :manip-input strategy))
  ;; save
  (save-variable)
  ;; plot
  (plot-all)
  ;; wait
  (read-line)
  (unless (analyze)
    (warn "failed classification~%")
    (return-from main-strategy nil))
  ;; plot
  (plot-all-class-data)
  )
 
