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
(load "package://hrpsys_gazebo_tutorials/euslisp/manip-obj-with-hand-print.l")
(load "package://hrpsys_gazebo_tutorials/euslisp/manip-obj-with-hand-plot.l")
(load "package://hrpsys_gazebo_tutorials/euslisp/manip-obj-with-hand-strategy.l")

(load "package://hrpsys_gazebo_tutorials/euslisp/hand-command-publisher.l")
(load "package://hrpsys_gazebo_tutorials/euslisp/drum_body.l")
(load "package://hrpsys_gazebo_tutorials/euslisp/metal_small_cart.l")


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
  (usage-print)
  (usage-plot)
  (usage-strategy)
  (usage-util)
  )

;; initialize ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun init
  (&key (model-num 4) (model-name "cupboard_wh") (force nil))

  (when (and (boundp '*manip-obj-with-hand-init-flag*) (not force))
    (init-variable)
    (return-from init nil))

  ;; init timer
  (setq *gazebo-timer* (init-gazebo-timer))

  ;; init model name
  (setq *model-name* model-name)

  ;; load controller parameter for hand
  (unix::system "rosparam load `rospack find hrpsys_gazebo_tutorials`/config/HRP3HAND_L.yaml HRP3HAND_L")
  (unix::system "rosparam load `rospack find hrpsys_gazebo_tutorials`/config/HRP3HAND_R.yaml HRP3HAND_R")

  ;; make gazebo model and plugin interface
  (setq *gplist* nil)
  (dotimes (i model-num)
    ;; add model to the gazebo world
    (unix::system (format nil "gzfactory spawn -f `rospack find hrpsys_gazebo_tutorials`/environment_models/~a/model~d.urdf -m ~a -x 0 -y 0 -z 1" *model-name* (+ i 1) *model-name*))

    ;; make gazebo plugin interface
    (eval (list 'setq (read-from-string (format nil "*gp~d*" (+ i 1))) (init-gazebo-plugin (format nil "obj~d" (+ i 1)))))
    (eval (list 'push-back (read-from-string (format nil "*gp~d*" (+ i 1))) '*gplist*)))

  ;; make hand controller interface
  (setq *hc* (instance hand-command-publisher-class :init))
  (sleep-gazebo 1)

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
    (let* (obj-tmp)
      (cond 
       ((equal *model-name* "cupboard_wh")
	(eval (list 'setq (read-from-string (format nil "*obj~d*" (+ i 1))) (make-73b2-cupboard-without-all-door)))
	(eval (list 'setq 'obj-tmp (read-from-string (format nil "*obj~d*" (+ i 1)))))
	(send *lhand* :newcoords (send obj-tmp :handle :name :left-front-handle1))
	(send *rhand* :newcoords (send obj-tmp :handle :name :right-front-handle1))
	)
       ((equal *model-name* "drum_wh")
	(eval (list 'setq (read-from-string (format nil "*obj~d*" (+ i 1))) (instance drum_body_gkmodel0_nil_link_geom0 :init)))
	(eval (list 'setq 'obj-tmp (read-from-string (format nil "*obj~d*" (+ i 1)))))
	(send obj-tmp :newcoords (make-coords))
	(send *lhand* :newcoords (make-coords :pos (v+ (send obj-tmp :worldpos) (float-vector 1800 -270 800)):rot (rpy-matrix pi -pi/2 0)))
	(send *rhand* :newcoords (make-coords :pos (v+ (send obj-tmp :worldpos) (float-vector 1800 410 800)):rot (rpy-matrix pi -pi/2 0)))
	)
       ((equal *model-name* "metal_small_cart_wh")
	(eval (list 'setq (read-from-string (format nil "*obj~d*" (+ i 1))) (make-metal-small-cart)))
	(eval (list 'setq 'obj-tmp (read-from-string (format nil "*obj~d*" (+ i 1)))))
	(send obj-tmp :newcoords (make-coords))
	(send *lhand* :newcoords (make-coords :pos (v+ (send obj-tmp :worldpos) (float-vector 1800 -270 800)):rot (rpy-matrix pi -pi/2 0)))
	(send *rhand* :newcoords (make-coords :pos (v+ (send obj-tmp :worldpos) (float-vector 1800 410 800)):rot (rpy-matrix pi -pi/2 0)))
	))
      ;; assoc hand
      (send obj-tmp :assoc *lhand*)
      (send obj-tmp :assoc *rhand*)
      (push-back obj-tmp *objlist*))
    )
  ;; make ground
  (setq *ground* (instance ground :init 5000 5000 450 450 "702-floor-tex.jpg" :name "702-ground"))
  ;; view models
  (unless (boundp '*pickview*)
    (pickview :no-menu t))
  (objects (append *arlist* *handlist* *objlist* (list *ground*)))

  ;; set hook-pose
  (set-hand-pose :hook-pose :arms)
  (sleep-gazebo 1)

  ;; init variables
  (init-variable)

  ;; init contact
  (init-contact)

  (setq *manip-obj-with-hand-init-flag* t)
  )

(defun init-cupboard (&rest args)
  (apply #'init :model-name "cupboard_wh" args))

(defun init-drum (&rest args)
  (apply #'init :model-name "drum_wh" args))

(defun init-cart (&rest args)
  (apply #'init :model-name "metal_small_cart_wh" args))


;; general main function ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun main-try
  (&key (manip-loop-num nil) (no-plot nil))

  ;; init
  (init)
  ;; loop
  (do-until-key
   (manip)
   (when (and manip-loop-num (> *manip-cnt* manip-loop-num))
     (return-from nil nil)))
  ;; save
  (save-variable)
  ;; plot
  (unless no-plot (plot-all))
  )

(defun main-train
  (&key (class-num 3) (no-play t) (no-analyze nil) (no-print nil) (no-plot nil))

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
    (unless no-print
      (print-feature-value :class-num i)
      (print-class-info :class-num i))
    ;; plot
    (unless no-plot 
      (plot-all-class-data :class-num i :save nil))
    ;; manip
    (unless no-play
      (manip-all-class-data :class-num i))
    )
  ;; print
  (print-class-info)
  (print-result)
  )

(defun main-test
  (&key (obj-num 0) (class-num 3) (no-plot nil))

  ;; init
  (init-variable-test)
  ;; load
  (load-variable)
  ;; analyze
  (unless (and (boundp '*all-class-data*)
	       (equal (length *all-class-data*) 3)
	       (equal (length (elt *all-class-data* 1)) class-num))
    (warn "analyze training data~%")
    (unless (analyze :class-num class-num)
      (warn "failed classification~%")
      (return-from main2 nil)))
  ;; manip
  (dolist (manip *manip-list*)
    (manip :obj-num obj-num :manip-input manip :no-save :test))
  ;; plot
  (unless no-plot 
    (plot-all :contact-data *contact-list-test* :2dcoord-data *2dcoord-list-test*))
  ;; test
  (calc-feature-value :contact-data *contact-list-test* :2dcoord-data *2dcoord-list-test* :no-save :test :no-print t)
  (let* ((nearest-class (get-nearest-class))
	 (obj-num-res (get-nearest-obj nearest-class)))
    ;; print
    (print-class-list)
    (warn "ans: ~a  res: ~a  correct: ~a~%" obj-num-res obj-num (equal obj-num-res obj-num)))
  )

(defun main-train-test
  (&key (manip-loop-num 15) (class-num 5))

  ;; try
  (main-try :manip-loop-num manip-loop-num :no-plot t)
  ;; train
  (main-train :class-num class-num :no-plot t)
  ;; test
  (dotimes (i (length *objlist*))
    (main-test :obj-num i :class-num class-num :no-plot t)))

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


;; special main function ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun main-train-cupboard
  ()

  ;; generate
  (generate-cupboard)
  ;; init
  (init :model-num 6)
  ;; try
  (main-try :manip-loop-num 15 :no-plot t))

(defun main-test-cupboard
  ()

  ;; generate
  (generate-cupboard :error-ratio 5)
  ;; init
  (init :model-num 6 :force t)
  ;; train
  (main-train :class-num 5 :no-plot t)
  ;; test
  (dotimes (i (length *objlist*))
    (main-test :obj-num i :class-num 5 :no-plot t)))
