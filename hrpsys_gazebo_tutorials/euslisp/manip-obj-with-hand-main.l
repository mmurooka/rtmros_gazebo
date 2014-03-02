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

(load "package://hrpsys_gazebo_tutorials/euslisp/manip-obj-with-hand-strategy.l")
(load "package://hrpsys_gazebo_tutorials/euslisp/manip-obj-with-hand-analize.l")
(load "package://hrpsys_gazebo_tutorials/euslisp/manip-obj-with-hand-plot.l")



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

(defun generate ()
  (let* ((obj-pp-list ;; pysical parameter list
	  (list
	   (list (cons :mass 5)
		 (cons :origin (float-vector 0 0 0.5))
		 (cons :inertia (float-vector 10 5.7411 0.788))
		 (cons :mu1 0.1)
		 (cons :mu2 0.1)
		 (cons :fdir1 (float-vector 1 0 0))
		 (cons :kp 1000000.0)
		 (cons :kd 100.0))
	   (list (cons :mass 10)
		 (cons :origin (float-vector 0 0 0.5))
		 (cons :inertia (float-vector 6.6875 5.7411 0.788))
		 (cons :mu1 0.1)
		 (cons :mu2 0.1))
	   (list (cons :mass 20)
		 (cons :origin (float-vector 0 0 0.1))
		 (cons :inertia (float-vector 6.6875 5.7411 0.788))
		 (cons :mu1 0.1)
		 (cons :mu2 0.1))
	   (list (cons :mass 20)
		 (cons :origin (float-vector 0 0 1))
		 (cons :inertia (float-vector 6.6875 5.7411 0.788))
		 (cons :mu1 0.1)
		 (cons :mu2 0.1))
	   (list (cons :mass 20)
		 (cons :origin (float-vector 0 0 0.5))
		 (cons :inertia (float-vector 6.6875 5.7411 0.788))
		 (cons :mu1 0.3)
		 (cons :mu2 0.3)
		 (cons :fdir1 (float-vector 1 0 0))
		 (cons :kp 1000000.0)
		 (cons :kd 100.0))
	   (list (cons :mass 20)
		 (cons :origin (float-vector 0 0 0.5))
		 (cons :inertia (float-vector 0.1 0.1 0.1))
		 (cons :mu1 1.0)
		 (cons :mu2 1.0)
		 (cons :fdir1 (float-vector 1 0 0))
		 (cons :kp 1000000.0)
		 (cons :kd 100.0))))
	 (cnt 0))
    (dolist (obj-pp obj-pp-list)
      (incf cnt)
      ;; set physical parameter to original urdf
      (cond
       ((and (cdr (assoc :fdir1 obj-pp)) (cdr (assoc :kp obj-pp)) (cdr (assoc :kd obj-pp)))
	(unix::system 
	 (format nil "sed -e 's@  </link>@    <inertial>\\n      <mass value=\"~f\" />\\n      <origin xyz=\"~f ~f ~f\" rpy=\"0 0 0\"/>\\n      <inertia ixx=\"~f\" ixy=\"0\" ixz=\"0\" iyy=\"~f\" iyz=\"0\" izz=\"~f\"/>\\n    </inertial>\\n  </link>\\n  <gazebo reference=\"nil_link\">\\n    <kp>~f</kp>\\n    <kd>~f</kd>\\n    <mu1>~f</mu1>\\n    <mu2>~f</mu2>\\n    <fdir1>~f ~f ~f</fdir1>\\n  </gazebo>@g' `rospack find hrpsys_gazebo_tutorials`/environment_models/cupboard_wh/cupboard_body.urdf.in > `rospack find hrpsys_gazebo_tutorials`/environment_models/cupboard_wh/cupboard_body~d.urdf"
		 (cdr (assoc :mass obj-pp)) (elt (cdr (assoc :origin obj-pp)) 0) (elt (cdr (assoc :origin obj-pp)) 1) (elt (cdr (assoc :origin obj-pp)) 2) (elt (cdr (assoc :inertia obj-pp)) 0) (elt (cdr (assoc :inertia obj-pp)) 1) (elt (cdr (assoc :inertia obj-pp)) 2) (cdr (assoc :kp obj-pp)) (cdr (assoc :kd obj-pp)) (cdr (assoc :mu1 obj-pp)) (cdr (assoc :mu2 obj-pp)) (elt (cdr (assoc :fdir1 obj-pp)) 0) (elt (cdr (assoc :fdir1 obj-pp)) 1) (elt (cdr (assoc :fdir1 obj-pp)) 2) cnt))
	)
       (t
	(unix::system 
	 (format nil "sed -e 's@  </link>@    <inertial>\\n      <mass value=\"~f\" />\\n      <origin xyz=\"~f ~f ~f\" rpy=\"0 0 0\"/>\\n      <inertia ixx=\"~f\" ixy=\"0\" ixz=\"0\" iyy=\"~f\" iyz=\"0\" izz=\"~f\"/>\\n    </inertial>\\n  </link>\\n  <gazebo reference=\"nil_link\">\\n    <mu1>~f</mu1>\\n    <mu2>~f</mu2>\\n  </gazebo>@g' `rospack find hrpsys_gazebo_tutorials`/environment_models/cupboard_wh/cupboard_body.urdf.in > `rospack find hrpsys_gazebo_tutorials`/environment_models/cupboard_wh/cupboard_body~d.urdf"
		 (cdr (assoc :mass obj-pp)) (elt (cdr (assoc :origin obj-pp)) 0) (elt (cdr (assoc :origin obj-pp)) 1) (elt (cdr (assoc :origin obj-pp)) 2) (elt (cdr (assoc :inertia obj-pp)) 0) (elt (cdr (assoc :inertia obj-pp)) 1) (elt (cdr (assoc :inertia obj-pp)) 2) (cdr (assoc :mu1 obj-pp)) (cdr (assoc :mu2 obj-pp)) cnt))
	))
      ;; set object name to original urdf
      (unix::system 
       (format nil "sed -i -e 's@<objname>obj</objname>@<objname>obj~d</objname>@g' `rospack find hrpsys_gazebo_tutorials`/environment_models/cupboard_wh/cupboard_body~d.urdf" cnt cnt))
      ;; set object name to original xacro
      (unix::system 
       (format nil "sed -e 's@cupboard_wh/cupboard_body.urdf@cupboard_wh/cupboard_body~d.urdf@g' `rospack find hrpsys_gazebo_tutorials`/environment_models/cupboard_wh/model.urdf.xacro.in > `rospack find hrpsys_gazebo_tutorials`/environment_models/cupboard_wh/model~d.urdf.xacro" cnt cnt))
      ;; generate urdf by xacro
      (unix::system 
       (format nil "rosrun xacro xacro.py `rospack find hrpsys_gazebo_tutorials`/environment_models/cupboard_wh/model~d.urdf.xacro > `rospack find hrpsys_gazebo_tutorials`/environment_models/cupboard_wh/model~d.urdf" cnt cnt))
      (warn "generate ~a~%" (format nil "model~d.urdf" cnt))
      )
    ))

(defun generate-drum ()
  (let* ((obj-pp-list ;; pysical parameter list
	  (list
	   (list (cons :mass 5)
		 (cons :origin (float-vector 2 0 0.2))
		 (cons :inertia (float-vector 0.1 0.1 0.1))
		 ;; (cons :inertia (float-vector 6.6875 5.7411 0.788))
		 (cons :mu1 0.1)
		 (cons :mu2 0.1))
	   (list (cons :mass 10)
		 (cons :origin (float-vector 2 0 0.2))
		 (cons :inertia (float-vector 0.5 0.5 0.5))
		 (cons :mu1 0.1)
		 (cons :mu2 0.1))
	   (list (cons :mass 20)
		 (cons :origin (float-vector 2 0 0.05))
		 (cons :inertia (float-vector 2.0 2.0 1.0))
		 (cons :mu1 0.1)
		 (cons :mu2 0.1))
	   (list (cons :mass 20)
		 (cons :origin (float-vector 2 0 0.5))
		 (cons :inertia (float-vector 2.0 2.0 1.0))
		 (cons :mu1 0.1)
		 (cons :mu2 0.1))
	   (list (cons :mass 20)
		 (cons :origin (float-vector 2 0 0.2))
		 (cons :inertia (float-vector 0.5 0.5 0.5))
		 (cons :mu1 0.3)
		 (cons :mu2 0.3)
		 (cons :fdir1 (float-vector 1 0 0))
		 (cons :kp 1000000.0)
		 (cons :kd 100.0))
	   (list (cons :mass 20)
		 (cons :origin (float-vector 2 0 0.2))
		 (cons :inertia (float-vector 0.1 0.1 0.1))
		 (cons :mu1 1.0)
		 (cons :mu2 1.0)
		 (cons :fdir1 (float-vector 1 0 0))
		 (cons :kp 1000000.0)
		 (cons :kd 100.0))))
	 (cnt 0))
    (dolist (obj-pp obj-pp-list)
      (incf cnt)
      ;; set physical parameter to original urdf
      (cond
       ((and (cdr (assoc :fdir1 obj-pp)) (cdr (assoc :kp obj-pp)) (cdr (assoc :kd obj-pp)))
	(unix::system 
	 (format nil "sed -e 's@  </link>@    <inertial>\\n      <mass value=\"~f\" />\\n      <origin xyz=\"~f ~f ~f\" rpy=\"0 0 0\"/>\\n      <inertia ixx=\"~f\" ixy=\"0\" ixz=\"0\" iyy=\"~f\" iyz=\"0\" izz=\"~f\"/>\\n    </inertial>\\n  </link>\\n  <gazebo reference=\"nil_link\">\\n    <kp>~f</kp>\\n    <kd>~f</kd>\\n    <mu1>~f</mu1>\\n    <mu2>~f</mu2>\\n    <fdir1>~f ~f ~f</fdir1>\\n  </gazebo>@g' `rospack find hrpsys_gazebo_tutorials`/environment_models/drum_wh/drum_body.urdf.in > `rospack find hrpsys_gazebo_tutorials`/environment_models/drum_wh/drum_body~d.urdf"
		 (cdr (assoc :mass obj-pp)) (elt (cdr (assoc :origin obj-pp)) 0) (elt (cdr (assoc :origin obj-pp)) 1) (elt (cdr (assoc :origin obj-pp)) 2) (elt (cdr (assoc :inertia obj-pp)) 0) (elt (cdr (assoc :inertia obj-pp)) 1) (elt (cdr (assoc :inertia obj-pp)) 2) (cdr (assoc :kp obj-pp)) (cdr (assoc :kd obj-pp)) (cdr (assoc :mu1 obj-pp)) (cdr (assoc :mu2 obj-pp)) (elt (cdr (assoc :fdir1 obj-pp)) 0) (elt (cdr (assoc :fdir1 obj-pp)) 1) (elt (cdr (assoc :fdir1 obj-pp)) 2) cnt))
	)
       (t
	(unix::system 
	 (format nil "sed -e 's@  </link>@    <inertial>\\n      <mass value=\"~f\" />\\n      <origin xyz=\"~f ~f ~f\" rpy=\"0 0 0\"/>\\n      <inertia ixx=\"~f\" ixy=\"0\" ixz=\"0\" iyy=\"~f\" iyz=\"0\" izz=\"~f\"/>\\n    </inertial>\\n  </link>\\n  <gazebo reference=\"nil_link\">\\n    <mu1>~f</mu1>\\n    <mu2>~f</mu2>\\n  </gazebo>@g' `rospack find hrpsys_gazebo_tutorials`/environment_models/drum_wh/drum_body.urdf.in > `rospack find hrpsys_gazebo_tutorials`/environment_models/drum_wh/drum_body~d.urdf"
		 (cdr (assoc :mass obj-pp)) (elt (cdr (assoc :origin obj-pp)) 0) (elt (cdr (assoc :origin obj-pp)) 1) (elt (cdr (assoc :origin obj-pp)) 2) (elt (cdr (assoc :inertia obj-pp)) 0) (elt (cdr (assoc :inertia obj-pp)) 1) (elt (cdr (assoc :inertia obj-pp)) 2) (cdr (assoc :mu1 obj-pp)) (cdr (assoc :mu2 obj-pp)) cnt))
	))
      ;; set object name to original urdf
      (unix::system 
       (format nil "sed -i -e 's@<objname>obj</objname>@<objname>obj~d</objname>@g' `rospack find hrpsys_gazebo_tutorials`/environment_models/drum_wh/drum_body~d.urdf" cnt cnt))
      ;; set object name to original xacro
      (unix::system 
       (format nil "sed -e 's@drum_wh/drum_body.urdf@drum_wh/drum_body~d.urdf@g' `rospack find hrpsys_gazebo_tutorials`/environment_models/drum_wh/model.urdf.xacro.in > `rospack find hrpsys_gazebo_tutorials`/environment_models/drum_wh/model~d.urdf.xacro" cnt cnt))
      ;; generate urdf by xacro
      (unix::system 
       (format nil "rosrun xacro xacro.py `rospack find hrpsys_gazebo_tutorials`/environment_models/drum_wh/model~d.urdf.xacro > `rospack find hrpsys_gazebo_tutorials`/environment_models/drum_wh/model~d.urdf" cnt cnt))
      (warn "generate ~a~%" (format nil "model~d.urdf" cnt))
      )
    ))



(defun try ()

  ;; init variables
  (setq *manip-list* nil)
  (setq *cont-st-list* nil)
  (setq *2d-coord-list* nil)
  (setq *manip-cnt* 0)
  ;; loop
  (do-until-key
   (try-once)
   (incf *manip-cnt*))
  ;; save log
  (dump-loadable-structure
   (format nil "log/log-~a.l"
	   (concatenate string (remove 58 (remove 32 (butlast (concatenate cons (unix:asctime (unix:localtime))))))))
   *manip-list* *cont-st-list* *2d-coord-list*)
  ;; plot graph
  ;;(plot-contact)
  ;;(plot-2d-coord)
  )

(defun try-once (&optional manip-input)

  (send-all *gplist* :add-force2 (float-vector 0 0 0) (float-vector 0 0 0))
  (send-all *gplist* :set-pose (make-coords :rpy (list pi 0 0)))
  (unix::usleep (* 100 1000))

  (let* ((loop-num 500)
	 (f1x 0.0) (f1y 0.0) (f1z 0.0)
	 (f2x 0.0) (f2y 0.0) (f2z 0.0)
	 (fmax1 30) (fmax2 10) (fmin1 -30) (fmin2 -10) fmax fmin dfmax dfmin
	 ;; (fmax 20) (fmin -20) (dfmax (/ fmax 10.0)) (dfmin (/ fmin 10.0))
	 manip-tmp cont-st-tmp 2d-coord-tmp
	 (fall-down-flag nil)
	 )
    (dotimes (i loop-num)
      (setq fmax (+ (random (float (- fmax1 fmax2))) fmax2))
      (setq fmin (+ (random (float (- fmin2 fmin1))) fmin1))
      (setq dfmax (/ fmax 10.0))
      (setq dfmin (/ fmin 10.0))
      (let* ((df1x (+ (random (float (- dfmax dfmin))) dfmin))
	     (df1y (+ (random (float (- dfmax dfmin))) dfmin))
	     (df1z (+ (random (float (- dfmax dfmin))) dfmin))
	     (df2x (+ (random (float (- dfmax dfmin))) dfmin))
	     (df2y (+ (random (float (- dfmax dfmin))) dfmin))
	     (df2z (+ (random (float (- dfmax dfmin))) dfmin)))
	(setq f1x (max (min (+ f1x df1x) fmax) fmin)
	      f1y (max (min (+ f1y df1y) fmax) fmin)
	      f1z (max (min (+ f1z df1z) fmax) fmin))
	(setq f2x (max (min (+ f2x df2x) fmax) fmin)
	      f2y (max (min (+ f2y df2y) fmax) fmin)
	      f2z (max (min (+ f2z df2z) fmax) fmin))
	(cond
	 (manip-input
	  (send-all *gplist* :add-force2 (elt (elt manip-input i) 0) (elt (elt manip-input i) 1)))
	 (t
	  (send-all *gplist* :add-force2 (float-vector f1x f1y f1z) (float-vector f2x f2y f2z))))
	;; (warning-message 2 "loop ~d/~d f1:~a f2:~a~%" i loop-num (float-vector f1x f1y f1z) (float-vector f2x f2y f2z))

	(update-coord-cupboard)
	(update-contact-cupboard)
	(when (equal (mod i 10) 0)
	  (let* ((l1 (* (+ (norm (float-vector f1x f1y f1z)) 2) 5))
		 (f11n (normalize-vector (v- (float-vector f1x f1y f1z))))
		 (f12n (v* f11n (float-vector 1 0 0)))
		 (f13n (v* f11n f12n))
		 (f1coord (make-coords :pos (send (send *lhand* :copy-worldcoords) :worldpos) :rot (matrix f11n f12n f13n)))
		 (l2 (* (+ (norm (float-vector f2x f2y f2z)) 2) 5))
		 (f21n (normalize-vector (v- (float-vector f2x f2y f2z))))
		 (f22n (v* f21n (float-vector 1 0 0)))
		 (f23n (v* f21n f22n))
		 (f2coord (make-coords :pos (send (send *rhand* :copy-worldcoords) :worldpos) :rot (matrix f21n f22n f23n))))
	    (send *ar1* :newcoords (send (send (send f1coord :copy-worldcoords) :rotate pi/2 :x) :translate (float-vector 0 l1 0) :local))
	    (send *ar2* :newcoords (send (send (send f2coord :copy-worldcoords) :rotate pi/2 :x) :translate (float-vector 0 l2 0) :local))
	    (send *pickview* :draw-objects :flush nil))
	  (draw-contact-cupboard)
	  (send *viewer* :flush))
	(unless (equal i 0) ;; I don't know why, but the first coord is not updated.
	  (push-back (list (float-vector f1x f1y f1z) (float-vector f2x f2y f2z)) manip-tmp)
	  (push-back (list (send *obj* :get :2d-pos) (send *obj* :get :2d-angle)) 2d-coord-tmp)
	  (push-back (send *obj* :get :cont-st) cont-st-tmp))
	(when (check-fall-down)
	  (setq fall-down-flag t)
	  (return-from nil nil))
	(x::window-main-one)
	(unix::usleep (round (* *dt* 1000 1000)))
	))
    (cond (fall-down-flag
	   (warn "~2d : detect falling down~%" *manip-cnt*))
	  (t
	   (let* ((res-eval-list (mapcar #'calc-motion-eval (send-all (send-all *gplist* :get-pose) :copy-worldcoords)))
		  (res-eval-ave (/ (reduce #'+ res-eval-list) (length res-eval-list))))
	     (warn "~2d : ~a : ~a~%" *manip-cnt* res-eval-list res-eval-ave)
	     (push-back (list *manip-cnt* res-eval-list res-eval-ave) manip-tmp))))
    (when fall-down-flag
      (setq manip-tmp (make-list (- loop-num 1) :initial-element (float-vector 0 0 0)))
      (setq cont-st-tmp (make-list (- loop-num 1) :initial-element :face))
      (setq 2d-coord-tmp (make-list (- loop-num 1) :initial-element (list (float-vector 0 0 0) 0))))
    (unless manip-input
      (push-back manip-tmp *manip-list*)
      (push-back cont-st-tmp *cont-st-list*)
      (push-back 2d-coord-tmp *2d-coord-list*))
    (send-all *gplist* :add-force2 (float-vector 0 0 0) (float-vector 0 0 0))
    ))

(defun calc-motion-eval (c)

  (let* ((x-dist (elt (send c :worldpos) 0))
	 (x-dist-eval (/ x-dist 1000.0))
	 (base-vec (float-vector 0 0 1))
	 (trans-base-vec (send c :rotate-vector base-vec))
	 (rp-angle (acos (v. base-vec trans-base-vec)))
	 (rp-angle-eval (if (< (abs rp-angle) (/ pi 10)) 0 (* (+ rp-angle 1) (+ rp-angle 1))))
	 (yaw-angle (elt (car (send (send (send c :copy-worldcoords) :rotate pi :z) :rpy-angle)) 0))
	 (yaw-angle-eval (if (< (abs yaw-angle) pi/2) 0 (abs yaw-angle))))
    (+ x-dist-eval (- rp-angle-eval) (- yaw-angle-eval))
    ))

(defun check-fall-down ;; return t when falling down
  (&optional (c (send *obj* :copy-worldcoords)))

  (let* ((rp-angle-thre (/ pi 5))
	 (base-vec (float-vector 0 0 1))
	 (trans-base-vec (send c :rotate-vector base-vec))
	 (rp-angle (abs (acos (v. base-vec trans-base-vec)))))
    ;; (warn "~a/~a" rp-angle rp-angle-thre)
    (> rp-angle rp-angle-thre)
    ))

