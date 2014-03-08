(load "euslib/demo/murooka/hrp2_carry/hrp2-pivot-obj-demo.l")
(load "package://hrpsys_gazebo_tutorials/euslisp/metal_small_cart.l")

(setq real t)
(setq wait t)

(warning-message 1 "[mnplrn] init~%")
(my-wait :wait t)

;; init hrp2
(setup-real :real real)
;;(init-real-for-carry :real real)
(send *hrp2* :fix-leg-to-coords (make-coords))

;; init cart
(load "package://hrpsys_gazebo_tutorials/euslisp/metal_small_cart.l")
(setq *cart* (make-metal-small-cart))
(send *cart* :newcoords (make-coords :pos (float-vector 500 0 0) :rpy (list pi 0 0)))

;; draw
(unless (boundp '*pickview*)
  (pickview :no-menu t))
(objects (list *hrp2* *cart*))
(send-all (mapcar #'(lambda (handle) (send *cart* handle)) (list :left-handle :right-handle)) :draw-on :flush t :size 200)
(send-all (send-all (send *hrp2* :arms :end-coords) :copy-worldcoords) :draw-on :flush t :size 200 :color #f(1 0 0))

;; init pose
(warning-message 1 "[mnplrn] init pose~%")
(my-wait :wait t)

(hrp2-reset-manip-pose :real real)
(let* ((grab-before-pose :push-before-pose))
  (cond ((consp grab-before-pose)
	 (mapcar #'(lambda (arm gp) (send *hrp2* arm :hand gp)) (list :larm :rarm) grab-before-pose))
	(t (send *hrp2* :arms :hand grab-before-pose))))

(cond ((equal real :sim) (send-angle-vector-to-real-hrp2 :base-time 1))
      (real (send-angle-vector-to-real-hrp2 :base-time 3)))
(send *pickview* :draw-objects)

;; reach pose
(warning-message 1 "[mnplrn] reach pose~%")
(my-wait :wait t)

(let* (ik-ret 
       (limb-list (list :larm :rarm))
       (target-coords (mapcar #'(lambda (handle) (send (send *cart* handle) :copy-worldcoords)) (list :left-handle :right-handle)))
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

(let* ((grab-before-pose :index-avoid-grasp-pose))
  (cond ((consp grab-before-pose)
	 (mapcar #'(lambda (arm gp) (send *hrp2* arm :hand gp)) (list :larm :rarm) grab-before-pose))
	(t (send *hrp2* :arms :hand grab-before-pose))))

(cond ((equal real :sim) (send-angle-vector-to-real-hrp2 :base-time 1))
      (real (send-angle-vector-to-real-hrp2 :base-time 3)))
(send *pickview* :draw-objects)


(defun init-real-for-mnplrn
  (&key (real nil))

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

;;(set-ref-fm :fx -50)
;; の姿勢で掴んで，
;;(set-ref-fm :fx 50)
;; で手前に引けば，
;; ロックありだと手前に傾いて，ロックなしだと手前に平行に引ける


;; todo
;;- 手の姿勢から物体の姿勢を推定する
;;- 持ち上げ
