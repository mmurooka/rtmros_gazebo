(load "euslib/demo/murooka/hrp2_carry/hrp2-carry-demo-all.l")
(load "package://hrpsys_gazebo_tutorials/euslisp/main.l")






(defun reach-cart
  (&key (real t) (wait t) (pull nil))

  ;; init
  (warning-message 1 "[mnplrn] init~%")
  (my-wait :wait t)

  ;; init hrp2
  (setup-real :real real)
  (init-real-for-mnplrn :real real)
  (if pull
      (send *hrp2* :fix-leg-to-coords (make-coords :pos #f(-200 0 0)))
    (send *hrp2* :fix-leg-to-coords (make-coords)))

  ;; init cart
  (setq *cart* (make-metal-small-cart))
  (add-grasp-handle-to-cart)
  (send *cart* :newcoords (make-coords :pos (float-vector 500 0 0) :rpy (list 0 0 0)))

  ;; draw
  (unless (boundp '*pickview*)
    (pickview :no-menu t))
  (objects (list *hrp2* *cart*))

  ;; init pose
  (warning-message 1 "[mnplrn] init pose~%")
  (my-wait :wait t)

  (hrp2-reset-manip-pose :real nil)
  (send *hrp2* :arms :move-end-pos (float-vector 0 0 50) :world)

  (cond ((equal real :sim) (send-angle-vector-to-real-hrp2 :base-time 1))
	(real (send-angle-vector-to-real-hrp2 :base-time 3)))
  (send *pickview* :draw-objects)

  ;; pre reach pose
  (warning-message 1 "[mnplrn] pre reach pose~%")
  (my-wait :wait t)

  (let* (ik-ret 
	 (limb-list (list :larm :rarm))
	 (trans-coord (list (make-coords :pos (float-vector 0 100 100)) (make-coords :pos (float-vector 0 -100 100))))
	 (handle-list (list :grasp-left-handle :grasp-right-handle))
	 (target-coords (mapcar #'(lambda (handle transc) (send (send (send *cart* handle) :copy-worldcoords) :transform transc)) handle-list trans-coord))
	 (links-dof (send *hrp2* :calc-target-joint-dimension
			  (mapcar #'(lambda (x) (send *hrp2* :link-list (send *hrp2* x :end-coords :parent)))
				  '(:rleg :lleg :rarm :larm))))
	 (nv (instantiate float-vector (+ 6 links-dof))) ;; generate vector for null-space calculation ;; 6 <= 6dof joint
	 (base-pos nil)
	 (foot-init-coords (send (send *hrp2* :foot-midcoords) :copy-worldcoords))
	 (foot-prints (list (list :rleg (list (float-vector 0 0 0) 0))
			    (list :lleg (list (float-vector 0 0 0) 0)))))
    (setq ik-ret
	  (fullbody-ik-origin
	   target-coords
	   foot-init-coords
	   :check-collision (and (substringp "HRP2" (string-upcase (send *hrp2* :name))) (equal (send (class *hrp2*) :name) 'hrp2))
	   :look-at-target nil
	   :arm limb-list :robot *hrp2*
	   :foot-prints foot-prints
	   :translation-axis (list t t)
	   :rotation-axis (list t t)
	   :thre '(10 10) :rthre '(10 10) :centroid-thre 50
	   :debug-view nil ;;:no-message
	   :null-space 
	   (if base-pos
	       ;; move down base-link by feedback base height
	       #'(lambda () (setf (elt nv 2) (* 0.001 (- base-pos (elt (send *hrp2* :worldpos) 2)))) nv)
	     nil)
	   )))
  (send *hrp2* :arms :hand :push-before-pose)
  (send *hrp2* :arms :hand :push-before-pose)

  (cond ((equal real :sim) (send-angle-vector-to-real-hrp2 :base-time 1))
	(real (send-angle-vector-to-real-hrp2 :base-time 3)))
  (send *pickview* :draw-objects)

  ;; reach pose
  (warning-message 1 "[mnplrn] reach pose~%")
  (my-wait :wait t)

  (let* (ik-ret 
	 (limb-list (list :larm :rarm))
	 (handle-list (list :grasp-left-handle :grasp-right-handle))
	 (target-coords (mapcar #'(lambda (handle) (send (send *cart* handle) :copy-worldcoords)) handle-list))
	 (links-dof (send *hrp2* :calc-target-joint-dimension
			  (mapcar #'(lambda (x) (send *hrp2* :link-list (send *hrp2* x :end-coords :parent)))
				  '(:rleg :lleg :rarm :larm))))
	 (nv (instantiate float-vector (+ 6 links-dof))) ;; generate vector for null-space calculation ;; 6 <= 6dof joint
	 (base-pos nil)
	 (foot-init-coords (send (send *hrp2* :foot-midcoords) :copy-worldcoords))
	 (foot-prints (list (list :rleg (list (float-vector 0 0 0) 0))
			    (list :lleg (list (float-vector 0 0 0) 0)))))
    (setq ik-ret
	  (fullbody-ik-origin
	   target-coords
	   foot-init-coords
	   :check-collision (and (substringp "HRP2" (string-upcase (send *hrp2* :name))) (equal (send (class *hrp2*) :name) 'hrp2))
	   :look-at-target nil
	   :arm limb-list :robot *hrp2*
	   :foot-prints foot-prints
	   :translation-axis (list t t)
	   :rotation-axis (list t t)
	   :thre '(10 10) :rthre '(10 10) :centroid-thre 50
	   :debug-view nil ;;:no-message
	   :null-space 
	   (if base-pos
	       ;; move down base-link by feedback base height
	       #'(lambda () (setf (elt nv 2) (* 0.001 (- base-pos (elt (send *hrp2* :worldpos) 2)))) nv)
	     nil)
	   )))

  (cond ((equal real :sim) (send-angle-vector-to-real-hrp2 :base-time 1))
	(real (send-angle-vector-to-real-hrp2 :base-time 3)))
  (send *pickview* :draw-objects)

  ;; grasp
  (warning-message 1 "[mnplrn] grasp pose~%")
  (my-wait :wait t)

  
  (set-ref-fm :fx 20) ;; move hand forward
  (unix::usleep (* 2 1000 1000))

  (send *hrp2* :arms :hand :angle-vector #f(90.0 89.1999 -10.0 20.0 -40.0 86.656))

  (cond ((equal real :sim) (send-angle-vector-to-real-hrp2 :base-time 1))
	(real (send-angle-vector-to-real-hrp2 :base-time 3)))

  (send *hrp2* :arms :hand :index-avoid-grasp-pose)
  (send *hrp2* :arms :hand :index-avoid-grasp-pose) ;; need for hand ik bug

  (cond ((equal real :sim) (send-angle-vector-to-real-hrp2 :base-time 1))
	(real (send-angle-vector-to-real-hrp2 :base-time 3)))
  (set-ref-fm :fx 0) ;; move hand original pos

  (send *pickview* :draw-objects)
  )


(defun release-cart
  (&key (real t) (wait t))

  ;; ungrasp
  (warning-message 1 "[mnplrn] ungrasp~%")
  (my-wait :wait t)

  (send *hrp2* :arms :hand :push-before-pose)
  (send *hrp2* :arms :hand :push-before-pose) ;; need for hand ik bug

  (cond ((equal real :sim) (send-angle-vector-to-real-hrp2 :base-time 1))
	(real (send-angle-vector-to-real-hrp2 :base-time 3)))
  (send *pickview* :draw-objects)

  ;; pre reach pose
  (warning-message 1 "[mnplrn] pre reach pose~%")
  (my-wait :wait t)

  (let* (ik-ret 
	 (limb-list (list :larm :rarm))
	 (trans-coord (list (make-coords :pos (float-vector 0 100 100)) (make-coords :pos (float-vector 0 -100 100))))
	 (handle-list (list :grasp-left-handle :grasp-right-handle))
	 (target-coords (mapcar #'(lambda (handle transc) (send (send (send *cart* handle) :copy-worldcoords) :transform transc)) handle-list trans-coord))
	 (links-dof (send *hrp2* :calc-target-joint-dimension
			  (mapcar #'(lambda (x) (send *hrp2* :link-list (send *hrp2* x :end-coords :parent)))
				  '(:rleg :lleg :rarm :larm))))
	 (nv (instantiate float-vector (+ 6 links-dof))) ;; generate vector for null-space calculation ;; 6 <= 6dof joint
	 (base-pos nil)
	 (foot-init-coords (send (send *hrp2* :foot-midcoords) :copy-worldcoords))
	 (foot-prints (list (list :rleg (list (float-vector 0 0 0) 0))
			    (list :lleg (list (float-vector 0 0 0) 0)))))
    (setq ik-ret
	  (fullbody-ik-origin
	   target-coords
	   foot-init-coords
	   :check-collision (and (substringp "HRP2" (string-upcase (send *hrp2* :name))) (equal (send (class *hrp2*) :name) 'hrp2))
	   :look-at-target nil
	   :arm limb-list :robot *hrp2*
	   :foot-prints foot-prints
	   :translation-axis (list t t)
	   :rotation-axis (list t t)
	   :thre '(10 10) :rthre '(10 10) :centroid-thre 50
	   :debug-view nil ;;:no-message
	   :null-space 
	   (if base-pos
	       ;; move down base-link by feedback base height
	       #'(lambda () (setf (elt nv 2) (* 0.001 (- base-pos (elt (send *hrp2* :worldpos) 2)))) nv)
	     nil)
	   )))
  (send *hrp2* :arms :hand :push-before-pose)
  (send *hrp2* :arms :hand :push-before-pose)

  (cond ((equal real :sim) (send-angle-vector-to-real-hrp2 :base-time 1))
	(real (send-angle-vector-to-real-hrp2 :base-time 3)))
  (send *pickview* :draw-objects)

  ;; fin pose
  (warning-message 1 "[mnplrn] fin pose~%")
  (my-wait :wait t)

  (hrp2-reset-manip-pose :real nil)
  (send *hrp2* :arms :move-end-pos (float-vector 0 0 50) :world)

  (cond ((equal real :sim) (send-angle-vector-to-real-hrp2 :base-time 1))
	(real (send-angle-vector-to-real-hrp2 :base-time 3)))
  (send *pickview* :draw-objects)
  )




;; init ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun init-real-for-mnplrn
  (&key (real t))

  (unless real (return-from init-real-for-mnplrn nil))

  ;; seq
  ;;      (send *ci* :set-interpolation-method "wor")
  (send *ci* :set-interpolation-method :linear "wor")
  ;; auto-balancer
  (send *ci* :set-abc-param :use-force :no-force)
  (send *ci* :set-abc-param :S #f(1 1 1 0 0 0))
  (send *ci* :start-auto-balancer)
  ;; impedance
  (send *ci* :set-impedance-param :moment-gain 0.05)
  (send *ci* :set-impedance-param :axis-r :y)
  ;;      (send *ci* :set-impedance-param :mdk #f(0.3 0.7 2.0)) ;; hard
  ;;      (send *ci* :set-impedance-param :mdk #f(0.005 0.4 0.08)) ;; normal
  (let* ((hard-imp-param #f(0.6 0.6 1.0))
	 (normal-imp-param #f(0.005 0.4 0.08))
	 (imp-param-mmmdddkkk-p
	  (concatenate float-vector (flatten (mapcar #'(lambda (x y z) (list x y z)) 
						     (concatenate cons hard-imp-param)
						     (concatenate cons hard-imp-param)
						     (concatenate cons hard-imp-param)))))
	 (imp-param-mmmdddkkk-r
	  (concatenate float-vector (flatten (mapcar #'(lambda (x y z) (list x y z)) 
						     (concatenate cons normal-imp-param)
						     (concatenate cons normal-imp-param)
						     (concatenate cons normal-imp-param))))))
    (send *ci* :set-impedance-param :mmmdddkkk-p imp-param-mmmdddkkk-p)
    (send *ci* :set-impedance-param :mmmdddkkk-r imp-param-mmmdddkkk-r))
  (send *ci* :start-fix)
  (send *ci* :start-impedance :arms)
  (unless (equal real :sim)
    (send *ci* :stop-st)))

;; try ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun try-push-cart
  (&key (real t) (pull nil))

  (setq *contact-list-real* nil)
  (setq *2dcoord-list-real* nil)

  (if pull
      (send *hrp2* :fix-leg-to-coords (make-coords :pos #f(-200 0 0)))
    (send *hrp2* :fix-leg-to-coords (make-coords)))  (move-cart-to-hand :init t)
  (send *pickview* :draw-objects :flush nil)
  (update-contact-cart-one :obj *cart*)
  (draw-contact-cart-one :obj *cart* :flush t)

  (add-force-for-check-slide-rotate-for-cart :real real :pull pull)

  (move-cart-to-hand :init nil)
  (send *pickview* :draw-objects)
  )

;; detect ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun add-force-for-check-slide-rotate-for-cart
  (&key (real nil) (wait t) (pull nil))

  (unless real
    (warning-message 4 "[check-slide-rotate] skip in case of not real ~%")
    (return-from add-force-for-check-slide-rotate-for-cart))

  (let* (original-av
	 (tmp-arm-x-diff-thre 
	  (abs (- (round (/ (/ (apply '+ (mapcar #'(lambda (x) (elt x 2)) 
						 (send *hrp2* :arms :end-coords :worldpos))) 2) 6)) 25)))
	 (arm-x-diff-thre (min tmp-arm-x-diff-thre 150))
	 (arm-pitch-diff-thre 5.0)
	 (inc-arm-x (* (if pull -1 1) (/ tmp-arm-x-diff-thre 10.0)))
	 (force 0)
	 slide-or-rotate original-arm-x original-larm-pitch original-rarm-pitch)

    (labels ((get-arm-x
	      ()
	      (let* ((rarm-endcoord-pos
		      (send (trans-fmc2world (send (send *hrp2* :rarm :end-coords :copy-worldcoords) :translate #f(60 0 0))) :pos))
		     (larm-endcoord-pos
		      (send (trans-fmc2world (send (send *hrp2* :larm :end-coords :copy-worldcoords) :translate #f(60 0 0))) :pos))
		     (rarm-x (aref rarm-endcoord-pos 0))
		     (larm-x (aref larm-endcoord-pos 0))
		     (arm-x (/ (+ larm-x rarm-x) 2)))
		arm-x
		))
	     (get-arm-x-diff
	      (&key (printp nil))
	      (let* ((tmp-arm-x (get-arm-x)) arm-x-diff)
		(unless original-arm-x (setq original-arm-x tmp-arm-x))
		(setq arm-x-diff (- tmp-arm-x original-arm-x))
		(warning-message 3 "[check-slide-rotate] arm-x diff [~6,1f] thre [~6,1f] ~%" arm-x-diff arm-x-diff-thre)
		arm-x-diff
		))
	     (get-arm-pitch
	      ()
	      (let* ((rarm-endcoord-rpy 
		      (mapcar #'rad2deg 
			      (car (rpy-angle (send (trans-fmc2world (send *hrp2* :rarm :end-coords :copy-worldcoords)) :rot)))))
		     (larm-endcoord-rpy 
		      (mapcar #'rad2deg 
			      (car (rpy-angle (send (trans-fmc2world (send *hrp2* :larm :end-coords :copy-worldcoords)) :rot)))))
		     (rarm-pitch (elt rarm-endcoord-rpy 1))
		     (larm-pitch (elt larm-endcoord-rpy 1)))
		(list rarm-pitch larm-pitch)
		))
	     (get-arm-pitch-diff
	      (&key (printp nil))
	      (let* ((arm-pitch-list (get-arm-pitch))
		     (rarm-pitch-diff (- (elt arm-pitch-list 0) original-rarm-pitch))
		     (larm-pitch-diff (- (elt arm-pitch-list 1) original-larm-pitch))
		     (arm-pitch-diff (/ (+ rarm-pitch-diff larm-pitch-diff) 2.0)))
		(warning-message 5 "[check-slide-rotate] arm-pitch rdiff [~6,1f]  ldiff [~6,1f]  diff [~6,1f] thre [~6,1f] ~%"
				 rarm-pitch-diff larm-pitch-diff arm-pitch-diff arm-pitch-diff-thre)
		arm-pitch-diff
		)))

      ;; estimate force
      (if (equal real :sim)
	  (warning-message 5 "[check-slide-rotate] skip estimate force in rats_server ~%")
	(progn
	  (warning-message 2 "[check-slide-rotate] start estimate force ~%")
	  (require (format nil "~A/hrp2/plugins/estimate-force.l" (unix:getenv "CVSDIR")))
	  (unless (boundp '*ofoi*) (init-estimate-operational-force))
	  (setq force (elt (car
			    (my-estimate-operational-force-by-trying-moving :move-axis (if pull :-x :x) :keep-est-force t)) 0))
	  (let ((additional-force 5))
	    (set-ref-force :x (+ force (* (/ force (abs force)) additional-force)) :time 1000))
	  (warning-message 3 "[check-slide-rotate] estimate-force [~a] ~%" force)
	  (send *hrp2* :angle-vector original-av)
	  (send-angle-vector-to-real-hrp2 :base-time 3)))

      ;; set original angle vector and arm pitch
      ;;    (mapcar #'send-real-angle-to-model-hrp2-limb (remove :rleg (remove :lleg (send-all (send *hrp2* :limbs) :name))))
      (send-real-angle-to-model-hrp2)
      (setq original-av (send *hrp2* :angle-vector))
      (let ((arm-pitch-list (get-arm-pitch)))
	(setq original-rarm-pitch (elt arm-pitch-list 0))
	(setq original-larm-pitch (elt arm-pitch-list 1)))

      (move-cart-to-hand :init t) ;; [tmp for mnplrn]
      (update-contact-cart-one :obj *cart*)
      (draw-contact-cart-one :obj *cart* :flush t)

      (let ((msg (instance std_msgs::Float64MultiArray :init)))
	(send msg :data (list arm-x-diff-thre arm-pitch-diff-thre))
	(ros::ros-info "msg [~A]" (send msg :data))
	(ros::publish "slide_rotate_thre" msg))

      (let* ((dt 0.1) (dt-for-ik 1.0) (start-time-for-ik (get-time)) (start-foot-midcoords (send (send *hrp2* :foot-midcoords) :copy-worldcoords))
	     contact-tmp 2dcoord-tmp)
	(do-until-key
	 ;; renew robot model
	 ;;       (mapcar #'send-real-angle-to-model-hrp2-limb (remove :rleg (remove :lleg (send-all (send *hrp2* :limbs) :name))))
	 (send-real-angle-to-model-hrp2)
	 (send *hrp2* :fix-leg-to-coords start-foot-midcoords '(:rleg :lleg))
	 (move-cart-to-hand) ;; [tmp for mnplrn]
	 (send *pickview* :draw-objects :flush nil)
	 (update-contact-cart-one :obj *cart*)
	 (draw-contact-cart-one :obj *cart* :flush t)

	 ;; renew arm-x-diff, arm-pitch-diff and check over thre
	 (let* ((tmp-arm-x-diff (get-arm-x-diff :printp t))
		(tmp-arm-pitch-diff (get-arm-pitch-diff :printp t)))
	   (when (> (abs tmp-arm-x-diff) arm-x-diff-thre)
	     (warning-message 6
			      "[check-slide-rotate] arm-x-diff over [~6,1f > ~6,1f] ~%" tmp-arm-x-diff arm-x-diff-thre)
	     (return))
	   (when (> (abs tmp-arm-pitch-diff) arm-pitch-diff-thre)
	     (warning-message 6
			      "[check-slide-rotate] arm-pitch-diff over [~6,1f > ~6,1f] ~%" tmp-arm-pitch-diff arm-pitch-diff-thre)
	     (return)))

	 ;; solve ik to move arm in x direction
	 (when (> (- (get-time :init nil) start-time-for-ik) dt-for-ik)
	   (warning-message 2 "[check-slide-rotate] arm-x-diff ~6,1f inc-arm-x ~6,1f ~%" (get-arm-x-diff) inc-arm-x)
	   (setq start-time-for-ik (get-time :init nil))
	   (let ((ik-ret
		  (send *hrp2* :arms :move-end-pos (trans-world2fmc (float-vector inc-arm-x 0 0)) :world)))
	     ;; check IK fail
	     (when (some #'(lambda (x) (equal x nil)) ik-ret)
	       (warning-message 4 "[check-slide-rotate] IK fail ~%")
	       (return)
	       ))
	   (send-angle-vector-to-real-hrp2 :base-time dt-for-ik :smooth t :wait nil))
	 
	 ;; wait for loop
	 (unix::usleep (round (* dt 1000 1000)))

	 (push-back (send *cart* :get :contact) contact-tmp)
	 (push-back (list (send *cart* :get :2d-pos) (send *cart* :get :2d-angle)) 2dcoord-tmp)
	 )
	(push-back (list contact-tmp) *contact-list-real*)
	(push-back (list 2dcoord-tmp) *2dcoord-list-real*)
	)

      (let ((msg (instance std_msgs::Float64MultiArray :init)))
	(send msg :data (list 0))
	(ros::ros-info "msg [~A]" (send msg :data))
	(ros::publish "slide_rotate_thre" msg))

      ;; check slide or rotate
      (let* ((tmp-arm-x-diff (get-arm-x-diff))
	     (tmp-arm-pitch-diff (get-arm-pitch-diff)))
	(setq slide-or-rotate (if (> (abs tmp-arm-pitch-diff) arm-pitch-diff-thre) :rotate :slide))
	(warning-message 6 "[check-slide-rotate] arm-x diff [~6,2f] thre [~6,2f]  arm-pitch diff [~6,2f] thre [~6,2f] ~%"
			 tmp-arm-x-diff arm-x-diff-thre tmp-arm-pitch-diff arm-pitch-diff-thre)
	(warning-message 6 "[check-slide-rotate] slide-or-rotate [~a] force [~a] ~%"
			 slide-or-rotate force))
      
      ;; move arms to original pos
      (set-ref-fm :fx 0 :time 1500)
      (send *hrp2* :angle-vector original-av)
      (send-angle-vector-to-real-hrp2 :base-time 3)
      (send *pickview* :draw-objects)

      (list slide-or-rotate force))))

;; util ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun move-cart-to-hand
  (&key (init nil) (on-ground t))
  (dotimes (i 10)
    (let* ((default-handle-coord
	     (apply #'midcoords 0.5 (mapcar #'(lambda (h) (send (send (send *cart* h) :coords) :copy-worldcoords)) (list :grasp-left-handle :grasp-right-handle))))
	   (hand-coord
	    (apply #'midcoords 0.5 (mapcar #'(lambda (limb) (send (send *hrp2* limb :end-coords) :copy-worldcoords)) (list :larm :rarm))))
	   (init-hand-coord
	    (make-coords :pos (float-vector (elt (send hand-coord :pos) 0) (elt (send hand-coord :pos) 1) (elt (send default-handle-coord :pos) 2))
			 :rot (send default-handle-coord :rot)))
	   (handle-coord
	    (if init
		(apply #'midcoords 0.5 (mapcar #'(lambda (h) (send (send *cart* h) :copy-worldcoords)) (list :grasp-left-handle :grasp-right-handle)))
	      (send (send *cart* :get :grasp-mid-handle-mdf) :copy-worldcoords)
	      ))
	   (trans-diff-coord
	    (send (send handle-coord :transformation (if init init-hand-coord hand-coord)) :copy-worldcoords)
	    ))
      (send *cart* :transform trans-diff-coord)))

  (when init
    (let* ((grasp-mid-handle-mdf-coord
	    (make-cascoords :coords (apply #'midcoords 0.5 (mapcar #'(lambda (limb) (send (send *hrp2* limb :end-coords) :copy-worldcoords)) (list :larm :rarm)))
			    :name :grasp-mid-handle-mdf-coord)
	    ))
      (send *cart* :put :grasp-mid-handle-mdf grasp-mid-handle-mdf-coord)
      (send *cart* :assoc grasp-mid-handle-mdf-coord)))

  (when on-ground
    (let* ((bottom-vertices-handle
	    (list :front-left-vertex :front-right-vertex :rear-right-vertex :rear-left-vertex))
	   (bottom-vertices
	    (mapcar #'(lambda (h) (send (send *cart* h) :worldpos)) bottom-vertices-handle))
	   (bottom-vertices-z (mapcar #'(lambda (v) (elt v 2)) bottom-vertices))
	   (bottom-vertices-z-min (apply #'min bottom-vertices-z)))
      (send *cart* :translate (float-vector 0 0 (- bottom-vertices-z-min)) :world)
      )))



;; model ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun add-grasp-handle-to-cart
  ()

  (let* ((orig-handle-name (list :left-handle :right-handle))
	 (grasp-handle-name (list :grasp-left-handle :grasp-right-handle))
	 (handle-trans-diff
	  (list
	   (make-coords :pos (float-vector 0 25 75) :rpy (list -0.529099 -0.37603 -0.634775))
	   (make-coords :pos (float-vector 0 -25 75) :rpy (list 0.515513 -0.373694 0.671314))))
	 (grasp-handle-cascoords
	  (mapcar
	   #'(lambda (td ohn ghn) (make-cascoords :coords (send (send (send *cart* ohn) :copy-worldcoords) :transform td) :name ghn))
	   handle-trans-diff orig-handle-name grasp-handle-name)))
    (mapcar #'(lambda (ghcc) (send *cart* :assoc ghcc)) grasp-handle-cascoords)
    (mapcar #'(lambda (ghn ghcc) (send *cart* :put ghn ghcc)) grasp-handle-name grasp-handle-cascoords)
    ))

(defmethod metal-small-cart
  (:grasp-left-handle () (send self :get :grasp-left-handle))
  (:grasp-right-handle () (send self :get :grasp-right-handle)))

;; analyze ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun plot-2dcoord-real
  ()
  (plot-2dcoord :data *2dcoord-list-real*))
(defun plot-contact-real
  ()
  (plot-contact :data *contact-list-real*))

(defun calc-feature-value-real
  ()
  (car (calc-feature-value-one
	(car (car *contact-list-real*))
	(car (car *2dcoord-list-real*)))))
(defun get-nearest-class-real
  ()
  (get-nearest-class-one
   (calc-feature-value-real)))
