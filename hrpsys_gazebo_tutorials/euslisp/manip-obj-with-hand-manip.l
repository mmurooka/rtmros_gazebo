#!/usr/bin/env roseus

#-:jsk
(jsk)
#-:rbrain-basic
(rbrain)


(load "package://hrpsys_gazebo_tutorials/euslisp/manip-obj-with-hand-util.l")


(defun usage-manip
  ()

  (warn ";; manip ;;;;;;;;;;;;;;;;;~%")
  (warn "(manip)~%")
  (warn "(manip :manip-input (elt *manip-list* 0)) ;; play log manip~%")
  )


;; manip ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun manip (&key (manip-input nil) (no-save nil))

  ;; init object pose
  (send-all *gplist* :add-force2 (float-vector 0 0 0) (float-vector 0 0 0))
  (send-all *gplist* :set-pose (make-coords :rpy (list pi 0 0)))
  (unix::usleep (* 100 1000))

  (let* (loop-num ;; loop num
	 manip-tmp contact-tmp 2dcoord-tmp
	 (fall-down-flag nil) (float-flag nil)
	 flist
	 (f1 (float-vector 0 0 0)) (f2 (float-vector 0 0 0))
	 (m1 (float-vector 0 0 0)) (m2 (float-vector 0 0 0)))
    (cond (manip-input (setq loop-num (length manip-input)))
	  (t (setq loop-num *manip-loop-num*)))
    (dotimes (i loop-num)
      ;; update force
      (cond
       (manip-input
	(setq f1 (elt (elt manip-input i) 0)
	      f2 (elt (elt manip-input i) 1))
	(when (equal (length (elt manip-input i)) 4) ;; in the case manip-input includes force moment
	  (setq m1 (elt (elt manip-input i) 2)
		m2 (elt (elt manip-input i) 3))))
       (t
	(when (equal (mod i *update-force-loop-num*) 0)
	  (setq flist (get-random-force))
	  (setq f1 (elt flist 0)
		f2 (elt flist 1))
	  (warning-message 2 "loop ~d/~d f1:~a f2:~a m1:~a m2:~a~%" i loop-num f1 f2 m1 m2)
	  )))
      ;; add force
      (send-all *gplist* :add-force2 f1 f2 m1 m2)
      ;;(warning-message 2 "loop ~d/~d f1:~a f2:~a m1:~a m2:~a~%" i loop-num f1 f2 m1 m2)

      ;; update coord and contact and view them
      (update-coord-cupboard)
      (update-contact-cupboard)
      (when (equal (mod i 10) 0)
	(let* ((l1 (* (+ (norm f1) 2) 5))
	       (f11n (normalize-vector f1))
	       (f12n (v* f11n (float-vector 1 1 1)))
	       (f13n (v* f11n f12n))
	       (f1coord (make-coords :pos (send (send *lhand* :copy-worldcoords) :worldpos) :rot (send (send (send *lhand* :copy-worldcoords) :transform (make-coords :rot (transpose (matrix f11n f12n f13n)))) :rot)))
	       (l2 (* (+ (norm f2) 2) 5))
	       (f21n (normalize-vector f2))
	       (f22n (v* f21n (float-vector 1 1 1)))
	       (f23n (v* f21n f22n))
	       (f2coord (make-coords :pos (send (send *rhand* :copy-worldcoords) :worldpos) :rot (send (send (send *rhand* :copy-worldcoords) :transform (make-coords :rot (transpose (matrix f21n f22n f23n)))) :rot))))
	  (send *left-ar* :newcoords (send (send (send (send f1coord :copy-worldcoords) :rotate -pi/2 :z) :rotate 0 :x) :translate (float-vector 0 l1 0) :local))
	  (send *right-ar* :newcoords (send (send (send (send f2coord :copy-worldcoords) :rotate -pi/2 :z) :rotate 0 :x) :translate (float-vector 0 l2 0) :local))
	  (send *pickview* :draw-objects :flush nil))
	(draw-contact-cupboard)
	(send *viewer* :flush))
      (unless (equal i 0) ;; I don't know why, but the first coord is not updated.
	(push-back (list f1 f2) manip-tmp)
	(push-back (list (send *obj* :get :2d-pos) (send *obj* :get :2d-angle)) 2dcoord-tmp)
	(push-back (send *obj* :get :contact) contact-tmp))
      (unless (equal i 0)
	;; check if the object is floated
	(when (equal (send *obj* :get :contact) :nil)
	  (setq float-flag t)
	  (return-from nil nil))
	;; check falling down
	(when (check-fall-down)
	  (setq fall-down-flag t)
	  (return-from nil nil)))
      (x::window-main-one)
      (unix::usleep (round (* *manip-dt* 1000 1000)))
      )
    (cond (float-flag
	   (warn "~2d : detect floating~%" *manip-cnt*))
	  (fall-down-flag
	   (warn "~2d : detect falling down~%" *manip-cnt*))
	  (t
	   (warn "~2d : finished manipulation~%" *manip-cnt*)))
    ;; overwirte with the invalid value when falling down or float
    (when float-flag
      (setq manip-tmp (append manip-tmp (make-list (- (- loop-num 1) (length manip-tmp)) :initial-element (list (float-vector 0 0 0) (float-vector 0 0 0)))))
      (setq contact-tmp (append contact-tmp (make-list (- (- loop-num 1) (length contact-tmp)) :initial-element :nil)))
      (setq 2dcoord-tmp (append 2dcoord-tmp (make-list (- (- loop-num 1) (length 2dcoord-tmp))
						       :initial-element (list (concatenate float-vector (send *obj* :get :2d-pos) (float-vector 0))
									      (send *obj* :get :2d-angle))))))
    (when fall-down-flag
      (setq manip-tmp (append manip-tmp (make-list (- (- loop-num 1) (length manip-tmp)) :initial-element (list (float-vector 0 0 0) (float-vector 0 0 0)))))
      (setq contact-tmp (make-list (- loop-num 1) :initial-element :invalid))
      ;;(setq contact-tmp (append contact-tmp (make-list (- (- loop-num 1) (length contact-tmp)) :initial-element :invalid)))
      (setq 2dcoord-tmp (append 2dcoord-tmp (make-list (- (- loop-num 1) (length 2dcoord-tmp))
						       :initial-element (list (concatenate float-vector (send *obj* :get :2d-pos) (float-vector 0))
									      (send *obj* :get :2d-angle))))))
    (unless no-save
      (push-back manip-tmp *manip-list*)
      (push-back contact-tmp *contact-list*)
      (push-back 2dcoord-tmp *2dcoord-list*)
      (incf *manip-cnt*))
    (send-all *gplist* :add-force2 (float-vector 0 0 0) (float-vector 0 0 0))
    ))

(defun manip-all-class-data
  (&key (class-num nil))

  (dotimes (i (length (elt *all-class-data* 1)))
    (when class-num (setq i class-num))
    (let* ((manip-class-list
	    (mapcar #'(lambda (x) (cdr (assoc :manip x)))
		    (elt (elt *all-class-data* 1) i))))

      (dotimes (j (length manip-class-list))
	(warn "class: ~a/~a  manip: ~a/~a ~%"
	      (+ i 1) (length (elt *all-class-data* 1)) (+ j 1) (length manip-class-list))
	(manip :manip-input (elt manip-class-list j) :no-save t))
      ;; wait
      (read-line))
    (when class-num (return-from nil nil))
    ))

;; util ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; get random force
(defun get-random-force
  ()

  (let* ((fx-list (list -10 -5 -1 0 1 5 10 15 20 25))
	 (fy-list (list 0))
	 (fz-list (list -3 -2 -1 0 1 2 3)))
    (list
     (float-vector (elt fx-list (random (length fx-list)))
		   (elt fy-list (random (length fy-list)))
		   (elt fz-list (random (length fz-list))))
     (float-vector (elt fx-list (random (length fx-list)))
		   (elt fy-list (random (length fy-list)))
		   (elt fz-list (random (length fz-list)))))
    ))

(defun get-random-force-old
  ()

  (let* ((f1x 0.0) (f1y 0.0) (f1z 0.0)
	 (f2x 0.0) (f2y 0.0) (f2z 0.0)
	 (fmax1 30) (fmax2 10) (fmin1 -30) (fmin2 -10) fmax fmin dfmax dfmin
	 (f1 (float-vector f1x f1y f1z)) (f2 (float-vector f2x f2y f2z))
	 ;; (fmax 20) (fmin -20) (dfmax (/ fmax 10.0)) (dfmin (/ fmin 10.0))
	 )
    ;; set force
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
      (setq f1 (float-vector f1x f1y f1z))
      (setq f2 (float-vector f2x f2y f2z))
      (list f1 f2)
      )))

;; check falling down
(defun check-fall-down ;; return t when falling down
  (&optional (c (send *obj* :copy-worldcoords)))

  (let* ((rp-angle-thre (/ pi 10))
	 (base-vec (float-vector 0 0 1))
	 (trans-base-vec (send c :rotate-vector base-vec))
	 (rp-angle (abs (acos (v. base-vec trans-base-vec)))))
    ;; (warn "~a/~a" rp-angle rp-angle-thre)
    (> rp-angle rp-angle-thre)
    ))
