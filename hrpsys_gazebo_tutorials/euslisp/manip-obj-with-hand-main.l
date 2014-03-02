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

(defun update-coord-cupboard (&optional (model-id 0))
  ;; set gazebo model pose to eus model
  (let* ((gzb-pose (send (send (elt *gplist* model-id) :get-pose) :copy-worldcoords))
	 (trans-gzb-pose (send gzb-pose :translate #f(0 0 -500.5) :local))
	 (gzb-pos (send trans-gzb-pose :worldpos)))
    (send *obj* :newcoords trans-gzb-pose)
    ;; (send *obj* :newcoords (make-coords :pos (float-vector 0 0 (elt gzb-pos 2)) :rot (send gzb-pose :rot))) ;; ignore x,y motion
    ))

(defun update-contact-cupboard ()
  (let* ((bottom-face (send *obj* :faces))
	 (bottom-edges (send (elt bottom-face 7) :edges))
	 (bottom-vertices (mapcar #'(lambda (l) (car (send l :vertices))) bottom-edges))
	 (bottom-vertices-z (mapcar #'(lambda (v) (elt v 2)) bottom-vertices))
	 bottom-vcont bottom-lcont
	 ;;(contact-z-thre 1)
	 (contact-z-thre -0.5))
    (dotimes (i (length bottom-vertices-z))
      (push-back (< (elt bottom-vertices-z i) contact-z-thre) bottom-vcont)
      )
    (dotimes (i (length bottom-vertices-z))
      (push-back (and (elt bottom-vcont i) (elt bottom-vcont (mod (+ i 1) 4))) bottom-lcont)
      )
    (print bottom-vertices-z)
    (send *obj* :put :vcont bottom-vcont)
    (send *obj* :put :lcont bottom-lcont)
    (send *obj* :put :fcont (not (position nil bottom-vcont)))
    (send *obj* :put :2d-pos (scale 0.001 (scale (/ 1.0 (length bottom-vertices)) (reduce #'v+ bottom-vertices))))
    (send *obj* :put :2d-angle (elt (car (send (send (send *obj* :copy-worldcoords) :rotate pi :z) :rpy-angle)) 0))
    (cond ((send *obj* :get :fcont) (send *obj* :put :cont-st :face))
	  ((position t (send *obj* :get :lcont)) (send *obj* :put :cont-st :line))
	  ((position t (send *obj* :get :vcont)) (send *obj* :put :cont-st :vertex))
	  (t (send *obj* :put :cont-st :nil)))
    ))

(defun draw-contact-cupboard ()
  (let* ((bottom-face (send *obj* :faces))
	 (bottom-edges (send (elt bottom-face 7) :edges))
	 (bottom-vertices (mapcar #'(lambda (l) (car (send l :vertices))) bottom-edges)))
    (dotimes (i (length bottom-vertices))
      (when (elt (send *obj* :get :vcont) i)
	(send (elt bottom-vertices i) :draw-on :color #f(1 0 0) :width 5))
      (when (elt (send *obj* :get :lcont) i)
	(send (elt bottom-edges i) :draw-on :color #f(1 0 0) :width 5))
      ))
  (send *pickview* :viewer :viewsurface
	:string 10 20 (string (send *obj* :get :cont-st)))
  )

(defun update-draw-contact-loop ()
  (do-until-key
   (update-coord-cupboard)
   (update-contact-cupboard)
   (send *pickview* :draw-objects :flush nil)
   (draw-contact-cupboard)
   (send *viewer* :flush)
   (x::window-main-one)))

(defun set-thread-update-draw-contact ()
  (sys:make-thread 1)
  (sys:thread #'update-draw-contact-loop))

(defun pull-slide ()
  (send-all *gplist* :set-pose (make-coords :rpy (list pi 0 0)))
  (send-all *gplist* :add-force2 (float-vector 10 0 2) (float-vector 10 0 2))
  (update-draw-contact-loop)
  (send-all *gplist* :add-force2 (float-vector 0 0 0) (float-vector 0 0 0))
  )

(defun push-slide ()
  (send-all *gplist* :set-pose (make-coords :rpy (list pi 0 0)))
  (send-all *gplist* :add-force2 (float-vector -5 0 -5) (float-vector -5 0 -5))
  (update-draw-contact-loop)
  (send-all *gplist* :add-force2 (float-vector 0 0 0) (float-vector 0 0 0))
  )

(defun pull-tilt ()
  (send-all *gplist* :set-pose (make-coords :rpy (list pi 0 0)))
  (dotimes (i 4)
    (pull-tilt-right)
    (pull-tilt-left))
  )

(defun pull-tilt-right ()
  ;;(send-all *gplist* :add-force2 (float-vector -10 0 -2) (float-vector 22 0 2))
  (send-all *gplist* :add-force2 (float-vector -10 0 -2) (float-vector 20 1 2))
  (update-draw-contact-loop)
  (send-all *gplist* :add-force2 (float-vector 0 0 0) (float-vector 0 0 0))
  )

(defun pull-tilt-left ()
  ;;(send-all *gplist* :add-force2 (float-vector 22 0 2) (float-vector -10 0 -2))
  (send-all *gplist* :add-force2 (float-vector 20 -1 2) (float-vector -10 0 -2))
  (update-draw-contact-loop)
  (send-all *gplist* :add-force2 (float-vector 0 0 0) (float-vector 0 0 0))
  )

(defun push-tilt ()
  (send-all *gplist* :set-pose (make-coords :rpy (list pi 0 0)))
  (dotimes (i 4)
    (push-tilt-right)
    (push-tilt-left))
  )

(defun push-tilt-right ()
  (send-all *gplist* :add-force2 (float-vector -10 0 2) (float-vector 20 0 -2))
  (update-draw-contact-loop)
  (send-all *gplist* :add-force2 (float-vector 0 0 0) (float-vector 0 0 0))
  )

(defun push-tilt-left ()
  (send-all *gplist* :add-force2 (float-vector 20 0 -2) (float-vector -10 0 2))
  (update-draw-contact-loop)
  (send-all *gplist* :add-force2 (float-vector 0 0 0) (float-vector 0 0 0))
  )

(defun lift-up ()
  (send-all *gplist* :set-pose (make-coords :rpy (list pi 0 0)))
  (send-all *gplist* :add-force2 (float-vector 25 0 0) (float-vector 25 0 0) (float-vector 0 -6.7 0) (float-vector 0 -6.7 0))
  ;;(send-all *gplist* :add-force2 (float-vector 20 0 0) (float-vector 20 0 0) (float-vector 0 -6 0) (float-vector 0 -6 0))
  (update-draw-contact-loop)
  (send-all *gplist* :add-force2 (float-vector 0 0 0) (float-vector 0 0 0))
  )



(defun calc-feature-value
  (&optional (cont-st (car *cont-st-list*)) (2d-coord (car *2d-coord-list*)))

  (let* ((fv-yaw-vel (make-list 5 :initial-element 0))
	 (fv-d-vel (make-list 5 :initial-element 0))
	 (fv-cor-xy-yaw (make-list 5 :initial-element 0)))
    (dotimes (i (length cont-st))
      (unless (equal i 0)
	(let* ((cont (elt cont-st i))
	       (cont-num
		(cond ((equal cont :face) 3)
		      ((equal cont :line) 2)
		      ((equal cont :vertex) 1)
		      ((equal cont :nil) 0)))
	       (x-crnt (elt (car (elt 2d-coord i)) 0))
	       (x-prev (elt (car (elt 2d-coord (- i 1))) 0))
	       (y-crnt (elt (car (elt 2d-coord i)) 1))
	       (y-prev (elt (car (elt 2d-coord (- i 1))) 1))
	       (xy-vel (float-vector (- x-crnt x-prev) (- y-crnt y-prev)))
	       (d-vel (abs (/ (norm xy-vel) *dt*)))
	       (d-vel-thre 1)
	       (xy-unit-vel (if (< d-vel d-vel-thre) (float-vector 0 0) (normalize-vector xy-vel)))
	       (yaw-crnt (cadr (elt 2d-coord i)))
	       (yaw-prev (cadr (elt 2d-coord (- i 1))))
	       (yaw-vel (abs (/ (- yaw-crnt yaw-prev) *dt*)))
	       (yaw-unit-vel (float-vector (cos yaw-crnt) (sin yaw-crnt)))
	       (cor-xy-yaw (v. xy-unit-vel yaw-unit-vel)))

	  ;; (print (list yaw-vel d-vel xy-unit-vel yaw-unit-vel)) ;; for debug
	  (setf (elt fv-yaw-vel cont-num) (+ (elt fv-yaw-vel cont-num) yaw-vel))
	  (setf (elt fv-d-vel cont-num) (+ (elt fv-d-vel cont-num) d-vel))
	  (setf (elt fv-cor-xy-yaw cont-num) (+ (elt fv-cor-xy-yaw cont-num) cor-xy-yaw))
	  (setf (elt fv-yaw-vel 4) (+ (elt fv-yaw-vel 4) yaw-vel))
	  (setf (elt fv-d-vel 4) (+ (elt fv-d-vel 4) d-vel))
	  (setf (elt fv-cor-xy-yaw 4) (+ (elt fv-cor-xy-yaw 4) cor-xy-yaw))
	  )))

    ;; (print (list fv-yaw-vel fv-d-vel fv-cor-xy-yaw)) ;; for debug
    (concatenate float-vector (append fv-yaw-vel fv-d-vel (list 0 0) (subseq fv-cor-xy-yaw 2)))
    ))

(defun analyze (&key (class-num 5))
  (calc-feature-value-all :output nil)
  (classify-feature-value :class-num class-num))

(defun calc-feature-value-all (&key (output t))
  (setq *fv-list* nil)
  (dotimes (i (length *cont-st-list*))
    (let*
	((cont-st (elt *cont-st-list* i))
	 (2d-coord (elt *2d-coord-list* i)))
      (push-back (calc-feature-value cont-st 2d-coord) *fv-list*)))

  (when output
    (dolist (fv *fv-list*)
      (warn "----------------------------------------------------------------~%")
      (warn " ~10a  ~16a  ~16a  ~16a~%"
	    "cont\\fv" "fv-yaw-vel" "fv-d-vel" "fv-cor-xy-yaw")
      (warn "----------------------------------------------------------------~%")
      (dotimes (i 5)
	(warn " ~10a  ~16a  ~16a  ~16a~%"
	      (case i (0 :nil) (1 :vertex) (2 :line) (3 :face) (4 :sum)) (elt fv i) (elt fv (+ i 5)) (elt fv (+ i 10)))))
    (warn "----------------------------------------------------------------~%"))
  )

(defun classify-feature-value (&key (class-num 3))
  ;;(load "/home/murooka/prog/jsk-ros-pkg-unreleased.bak.0218/jsk_rosmake_libs/euslib/jsk/lmeds.l")
  (setq *all-orig-data* nil)
  (dotimes (i (length *fv-list*))
    (push-back (list (cons :manip (elt *manip-list* i)) (cons :cont-st (elt *cont-st-list* i)) (cons :2d-coord (elt *2d-coord-list* i)) (cons :fv (elt *fv-list* i)))
	       *all-orig-data*))
  (setq *all-class-data* (k-means *all-orig-data* class-num :key #'(lambda (x) (cdr (assoc :fv x))) :debug nil))
  nil)

(defun plot-all-class-data ()
  (dotimes (i (length (elt *all-class-data* 1)))
    ;; plot
    (plot-contact :data (mapcar #'(lambda (x) (cdr (assoc :cont-st x))) (elt (elt *all-class-data* 1) i)))
    (plot-2d-coord :data (mapcar #'(lambda (x) (cdr (assoc :2d-coord x))) (elt (elt *all-class-data* 1) i)))
    (warn "plotting ~d data ~d / ~d. press any key for next plotting. ~%"
	  (length (mapcar #'(lambda (x) (cdr (assoc :cont-st x))) (elt (elt *all-class-data* 1) i))) (+ i 1) (length (elt *all-class-data* 1)))
    ;; wait
    (read-line)
    ;; save
    (save-plot-contact (format nil "fig/cont-st-graph-~a.png" i))
    (save-plot-2d-coord (format nil "fig/2d-coord-graph-~a.png" i))
    ;; wait
    (read-line)
    ))


(defun save-plot-contact (&optional (filename "fig/cont-st-graph-tmp.png"))
  (send *gnuplot-cont-st* :command "set terminal png")
  (send *gnuplot-cont-st* :command (format nil "set output \"~a\"" filename))
  (send *gnuplot-cont-st* :command "replot")
  (send *gnuplot-cont-st* :command "set terminal wxt")
  )

(defun save-plot-2d-coord (&optional (filename "fig/2d-coord-graph-tmp.png"))
  (send *gnuplot-2d-coord* :command "set terminal png")
  (send *gnuplot-2d-coord* :command (format nil "set output \"~a\"" filename))
  (send *gnuplot-2d-coord* :command "replot")
  (send *gnuplot-2d-coord* :command "set terminal wxt")
  )

(defun plot-contact (&key (data *cont-st-list*) (wait nil))
  (let* ((plot-file-name "log/tmp-log-for-plot-contact.txt")
	 (plot-log-stream
	  (open plot-file-name :direction :output)))
    (dotimes (i (length (car data)))
      (format plot-log-stream "~a" i)
      (dotimes (j (length data))
	(let* ((cont (elt (elt data j) i))
	       (cont-num
		(cond ((equal cont :face) 3)
		      ((equal cont :line) 2)
		      ((equal cont :vertex) 1)
		      ((equal cont :nil) 0))))
	  (format plot-log-stream " ~a" cont-num)))
      (format plot-log-stream "~%")
      )
    (close plot-log-stream)
    (send *gnuplot-cont-st* :command "set key outside")
    (send *gnuplot-cont-st* :command "set yrange [3.5:-0.5]")
    (send *gnuplot-cont-st* :command (format nil "plot \"~a\" u 1:2 with line lw 5 notitle" plot-file-name))
    ;;(send *gnuplot-cont-st* :command (format nil "plot \"~a\" u 1:2 with line lw 5 title \"1\"" plot-file-name))
    (when (> (length data) 1)
      (dotimes (j (length data))
	(unless (equal j 0)
	  (when wait
	    (warn "press any key for plotting next line~%")
	    (read-line))
	  (send *gnuplot-cont-st* :command (format nil "replot \"~a\" u 1:~a with line lw 5 notitle" plot-file-name (+ j 2)))
	  ;;(send *gnuplot-cont-st* :command (format nil "replot \"~a\" u 1:~a with line lw 5 title \"~a\"" plot-file-name (+ j 2) (+ j 1)))
	  )))
    ;;(warn "press [Y] key for delete plotting window~%")
    ;;(while (not (y-or-n-p)))
    ;;(send *gnuplot-cont-st* :quit)
    ))

(defun plot-2d-coord (&key (data *2d-coord-list*) (wait nil))
  (let* ((plot-file-name "log/tmp-log-for-plot-2d.txt")
	 (plot-log-stream
	  (open plot-file-name :direction :output))
	 (plot-vect-interval 20))
    (dotimes (i (length (car data)))
      (format plot-log-stream "~a" i)
      (dotimes (j (length data))
	(let* ((2d-coord-tmp (elt (elt data j) i))
	       (x-tmp (elt (car 2d-coord-tmp) 0))
	       (y-tmp (elt (car 2d-coord-tmp) 1))
	       (theta-tmp (cadr 2d-coord-tmp))
	       (vec-norm-tmp 0.2)
	       (x-tmp2 (* vec-norm-tmp (cos theta-tmp)))
	       (y-tmp2 (* vec-norm-tmp (sin theta-tmp))))
	  (format plot-log-stream " ~a ~a ~a ~a" x-tmp y-tmp x-tmp2 y-tmp2)))
      (format plot-log-stream "~%")
      )
    (close plot-log-stream)
    (send *gnuplot-2d-coord* :command "set key outside")
    (send *gnuplot-2d-coord* :command "set grid")
    (send *gnuplot-2d-coord* :command "set xrange [-5:5]")
    (send *gnuplot-2d-coord* :command "set yrange [-5:5]")
    (send *gnuplot-2d-coord* :command (format nil "plot \"~a\" u ~a:~a with line lw 5 notitle" plot-file-name 2 3))
    (send *gnuplot-2d-coord* :command (format nil "replot \"~a\" u ~a:~a:~a:~a every ~a with vector notitle" plot-file-name 2 3 4 5 plot-vect-interval))
    ;;(send *gnuplot-2d-coord* :command (format nil "plot \"~a\" u ~a:~a with line lw 5 title \"1\"" plot-file-name 2 3))
    ;;(send *gnuplot-2d-coord* :command (format nil "replot \"~a\" u ~a:~a:~a:~a every ~a with vector title \"1\"" plot-file-name 2 3 4 5 plot-vect-interval))
    (when (> (length data) 1)
      (dotimes (j (length data))
    	(unless (equal j 0)
    	  (when wait
    	    (warn "press any key for plotting next line~%")
    	    (read-line))
    	  (send *gnuplot-2d-coord* :command (format nil "replot \"~a\" u ~a:~a with line lw 5 notitle"
				    plot-file-name (+ (* j 4) 2) (+ (* j 4) 3)))
	  (send *gnuplot-2d-coord* :command (format nil "replot \"~a\" u ~a:~a:~a:~a every ~a with vector notitle"
				    plot-file-name (+ (* j 4) 2) (+ (* j 4) 3) (+ (* j 4) 4) (+ (* j 4) 5) plot-vect-interval))
    	  ;; (send *gnuplot-2d-coord* :command (format nil "replot \"~a\" u ~a:~a with line lw 5 title \"~a\""
	  ;; 			    plot-file-name (+ (* j 4) 2) (+ (* j 4) 3) (+ j 1)))
	  ;; (send *gnuplot-2d-coord* :command (format nil "replot \"~a\" u ~a:~a:~a:~a every ~a with vector title \"~a\""
	  ;; 			    plot-file-name (+ (* j 4) 2) (+ (* j 4) 3) (+ (* j 4) 4) (+ (* j 4) 5) plot-vect-interval (+ j 1)))
    	  )))
    ;;(warn "press [Y] key for delete plotting window~%")
    ;;(while (not (y-or-n-p)))
    ;;(send *gnuplot-2d-coord* :quit)
    ))

(defmethod gazebo-plugin-obj-operate
  ;; initialize
  (:init
   (&optional (_objname ""))
   (setq objname _objname)
   (ros::roseus "GazeboPluginUtilsEus")
   (ros::rate 100)		   
   (send self :init-add-force2)
   (send self :init-set-vel)
   (send self :init-get-vel)
   (send self :init-set-vel)
   (send self :update))
  ;; add-froce plugin ;;;;;;
  (:init-add-force2
   ()
   (ros::roseus "AddForceEus")
   (ros::advertise (format nil "~a~a" objname "/AddForcePlugin/ForceCommand1") geometry_msgs::wrench 1)
   (ros::advertise (format nil "~a~a" objname "/AddForcePlugin/ForceCommand2") geometry_msgs::wrench 1))
  (:add-force2-1
   (_f _t)
   (let* ((msg (instance geometry_msgs::wrench :init)))
     (send msg :force :x (elt _f 0))
     (send msg :force :y (elt _f 1))
     (send msg :force :z (elt _f 2))
     (send msg :torque :x (elt _t 0))
     (send msg :torque :y (elt _t 1))
     (send msg :torque :z (elt _t 2))
     (ros::publish (format nil "~a~a" objname "/AddForcePlugin/ForceCommand1") msg)))
  (:add-force2-2
   (_f _t)
   (let* ((msg (instance geometry_msgs::wrench :init)))
     (send msg :force :x (elt _f 0))
     (send msg :force :y (elt _f 1))
     (send msg :force :z (elt _f 2))
     (send msg :torque :x (elt _t 0))
     (send msg :torque :y (elt _t 1))
     (send msg :torque :z (elt _t 2))
     (ros::publish (format nil "~a~a" objname "/AddForcePlugin/ForceCommand2") msg)))
  (:add-force2
   (_f1 _f2 &optional (_t1 (float-vector 0 0 0)) (_t2 (float-vector 0 0 0)))
   (when _f1 (send self :add-force2-1 _f1 _t1))
   (when _f2 (send self :add-force2-2 _f2 _t2)))
  (:reset-force2
   ()
   (send self :add-force2 (float-vector 0 0 0) (float-vector 0 0 0)))
  )

(defun set-hand-pose (&optional (pose :hook-pose) (arm :arms))
  (when (or (equal arm :larm) (equal arm :lhand) (equal arm :arms) (equal arm :hands))
    (send *lhand* pose))
  (when (or (equal arm :rarm) (equal arm :rhand) (equal arm :arms) (equal arm :hands))
    (send *rhand* pose))
  (cond 
   ((or (equal arm :larm) (equal arm :lhand))
    (send *hc* :send-command (list (send *lhand* :angle-vector) nil)))
   ((or (equal arm :rarm) (equal arm :rhand))
    (send *hc* :send-command (list nil (send *rhand* :angle-vector))))
   ((or (equal arm :arms) (equal arm :hands))
    (send *hc* :send-command (list (send *lhand* :angle-vector) (send *rhand* :angle-vector)))))
  )

(defun make-73b2-cupboard-without-all-door
  (&key (name "73b2-cupboard-without-door"))
  (let* ((d 353) (w 586) (h 1816)
	 (d/2 (/ d 2)) (w/2 (/ w 2)) (-w/2 (/ w -2))
	 (handle-offset 850)
	 c)
    (setq c (instance askul-cupboard :init d w h
		      :name name
		      :panel-heights (list 760 880
					   (+ 870 (* 230 1))
					   (+ 870 (* 230 2)))))
    (send c :rotate pi :z :local)
    (send c :add-handle (make-cascoords :pos (float-vector (+ (/ d 2) 180) (- (- (/ w 2)) 30) (- h handle-offset))
					:rot (rpy-matrix pi -pi/2 0) :name :left-front-handle1))
    (send c :add-handle (make-cascoords :pos (float-vector (+ (/ d 2) 180) (+ (+ (/ w 2)) 30) (- h handle-offset))
					:rot (rpy-matrix pi -pi/2 0) :name :right-front-handle1))
    (send c :add-handle (make-cascoords :pos (float-vector (/ d 2) (/ w 2) 0)
					:rot (unit-matrix 3) :name :left-front-edge))
    (send c :add-handle (make-cascoords :pos (float-vector (/ d 2) (- (/ w 2)) 0)
					:rot (unit-matrix 3) :name :right-front-edge))
    (send c :add-handle (make-cascoords :pos (float-vector (- (/ d 2)) (/ w 2) 0)
					:rot (unit-matrix 3) :name :left-rear-edge))
    (send c :add-handle (make-cascoords :pos (float-vector (- (/ d 2)) (- (/ w 2)) 0)
					:rot (unit-matrix 3) :name :right-rear-edge))
    c))

;; (send *gp* :add-force2 #f(100 0 0) #f(100 0 0)) ;; 上
;; (send *gp* :add-force2 #f(0 -200 0) #f(0 200 0)) ;; 内側
;; (send *gp* :add-force2 #f(0 0 -200) #f(0 0 -200)) ;; 前

;; (send *gp* :add-force2 #f(0 0 -18) #f(0 0 -18))
