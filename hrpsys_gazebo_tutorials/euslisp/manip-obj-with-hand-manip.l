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
(defun manip
  (&key (obj-num nil) (manip-num nil) (manip-input nil) (no-save nil))

  ;; when manip-num is specified, overwrite manip-input
  (when manip-num
    (setq manip-input (elt *manip-list* manip-num)))
  (let* ((obj nil) (gp nil) ;; obj and gp of specified num
	 (objlist-except nil) (gplist-except nil)) ;; obj and gp list which exclude specified num instance
    ;; set obj when obj-num is specified
    (when obj-num
      (eval (list 'setq 'obj (read-from-string (format nil "*obj~d*" (+ obj-num 1)))))
      (eval (list 'setq 'gp (read-from-string (format nil "*gp~d*" (+ obj-num 1)))))
      (setq objlist-except (remove obj *objlist*))
      (setq gplist-except (remove gp *gplist*)))
    ;; init object pose
    (cond
     (obj-num
      (send gp :add-force2 (float-vector 0 0 0) (float-vector 0 0 0))
      (send gp :set-pose (make-coords :rpy (list pi 0 0)))
      (send-all gplist-except :add-force2 (float-vector 0 0 0) (float-vector 0 0 0))
      (send-all gplist-except :set-pose (make-coords :pos (float-vector (* 1000 1000) 0 0) :rpy (list pi 0 0)))
      )
     (t
      (send-all *gplist* :add-force2 (float-vector 0 0 0) (float-vector 0 0 0))
      (send-all *gplist* :set-pose (make-coords :rpy (list pi 0 0)))
      ))
    ;; update first
    (sleep-gazebo 0.1)
    (update-obj-coord)
    (update-obj-contact)
    (update-ar-coord (float-vector 1 0 0) (float-vector 1 0 0))
    (send *pickview* :draw-objects :flush nil)
    (draw-obj-contact)
    (send *viewer* :flush)
    ;; loop
    (let* (loop-num ;; loop num
	   manip-tmp
	   (contact-tmp (make-list (length *objlist*)))
	   (2dcoord-tmp (make-list (length *objlist*)))
	   (fall-down-flag (make-list (length *objlist*)))
	   (float-flag (make-list (length *objlist*)))
	   (update-state-flag (make-list (length *objlist*) :initial-element t))
	   flist
	   (f1 (float-vector 0 0 0)) (f2 (float-vector 0 0 0))
	   (m1 (float-vector 0 0 0)) (m2 (float-vector 0 0 0))
	   (update-force-flag nil)
	   (draw-interval 10))
      (cond (manip-input (setq loop-num (length manip-input)))
	    (t (setq loop-num *manip-loop-num*)))
      ;; when obj-num is specified, set update-state-flag of unspecified objects as t
      (when obj-num
	(setq update-state-flag (make-list (length *objlist*) :initial-element nil))
	(setf (elt update-state-flag obj-num) t))
      (dotimes (i loop-num)
	;; init timer
	(sleep-gazebo-init *manip-dt*)
	;; update update-state-flag
	(setq update-state-flag
	      (mapcar #'(lambda (x y z) (and x (not (or y z)))) update-state-flag float-flag fall-down-flag))
	(when no-save ;; only when no-save because manip needs to be saved
	  (unless (reduce #'(lambda (x y) (or x y)) update-state-flag) ;; when all flag is nil
	    (warn "break from loop because update-state-flag is nil for all model~%")
	    (return-from nil nil)))
	;; update force
	(setq update-force-flag nil)
	(cond
	 (manip-input
	  (setq f1 (elt (elt manip-input i) 0)
		f2 (elt (elt manip-input i) 1))
	  (when (equal (length (elt manip-input i)) 4) ;; in the case manip-input includes force moment
	    (setq m1 (elt (elt manip-input i) 2)
		  m2 (elt (elt manip-input i) 3)))
	  (setq update-force-flag t))
	 (t
	  (when (equal (mod i *update-force-loop-num*) 0)
	    (setq flist (get-random-force))
	    (setq f1 (elt flist 0)
		  f2 (elt flist 1))
	    (setq update-force-flag t)
	    (warning-message 2 "loop ~d/~d f1:~a f2:~a m1:~a m2:~a~%" i loop-num f1 f2 m1 m2))))
	;; add force
	(when update-force-flag
	  (dotimes (i (length *gplist*))
	    (when (elt update-state-flag i)
	      (send (elt *gplist* i) :add-force2 f1 f2 m1 m2))))
	;; update coord and contact and view them
	(update-obj-coord :update-flag update-state-flag)
	(update-obj-contact :update-flag update-state-flag)
	;; update view
	(when (equal (mod i draw-interval) 0)
	  (update-ar-coord f1 f2 :update-flag update-state-flag)
	  (send *pickview* :draw-objects :flush nil)
	  (cond (obj-num (draw-obj-contact :update-flag update-state-flag))
		(t (draw-obj-contact)))
	  (send *viewer* :flush))
	(unless (equal i 0) ;; I don't know why, but the first coord is not updated.
	  ;; store manip, contact and 2dcoord
	  (push-back (list f1 f2) manip-tmp)
	  (dotimes (i (length *objlist*))
	    (when (elt update-state-flag i)
	      (let* ((obj-tmp (elt *objlist* i)))
		(push-back (send obj-tmp :get :contact) (elt contact-tmp i))
		(push-back (list (send obj-tmp :get :2d-pos) (send obj-tmp :get :2d-angle)) (elt 2dcoord-tmp i)))))
	  ;; check if the object is floated
	  (setq float-flag
		(mapcar #'(lambda (x y) (or x (equal y :nil)))
			float-flag (send-all *objlist* :get :contact)))
	  ;; check if the object is falling down
	  (setq fall-down-flag
		(mapcar #'(lambda (x y) (or x (check-fall-down y)))
			fall-down-flag (send-all *objlist* :copy-worldcoords))))
	(x::window-main-one)
	;; wait timer
	(sleep-gazebo-wait)
	;;(unix::usleep (round (* *manip-dt* 1000 1000)))
	)
      (warn "~2d : float : ~a  fall  : ~a~%" *manip-cnt* float-flag fall-down-flag)

      ;; overwirte with the invalid value when falling down or float
      (dotimes (i (length *objlist*))
	(let* ((obj (elt *objlist* i)))
	  (cond ((elt float-flag i)
		 (setf (elt contact-tmp i)
		       (append (elt contact-tmp i) (make-list (- (- loop-num 1) (length (elt contact-tmp i))) :initial-element :nil)))
		 (setf (elt 2dcoord-tmp i)
		       (append (elt 2dcoord-tmp i) (make-list (- (- loop-num 1) (length (elt 2dcoord-tmp i)))
							      :initial-element (list (concatenate float-vector (send obj :get :2d-pos) (float-vector 0))
										     (send obj :get :2d-angle))))))
		((elt fall-down-flag i)
		 (setf (elt contact-tmp i)
		       (make-list (- loop-num 1) :initial-element :invalid))
		 (setf (elt 2dcoord-tmp i)
		       (append (elt 2dcoord-tmp i) (make-list (- (- loop-num 1) (length (elt 2dcoord-tmp i)))
							      :initial-element (list (concatenate float-vector (send obj :get :2d-pos) (float-vector 0))
										     (send obj :get :2d-angle)))))))))
      ;; save manip data to global variable
      (cond ((equal no-save nil)
	     (push-back manip-tmp *manip-list*)
	     (push-back contact-tmp *contact-list*)
	     (push-back 2dcoord-tmp *2dcoord-list*)
	     (incf *manip-cnt*))
	    ((equal no-save :test)
	     (push-back (list (elt contact-tmp obj-num)) *contact-list-test*)
	     (push-back (list (elt 2dcoord-tmp obj-num)) *2dcoord-list-test*)
	     (incf *manip-cnt*)))
      (send-all *gplist* :add-force2 (float-vector 0 0 0) (float-vector 0 0 0))
      ))
  nil)

(defun manip-all-class-data
  (&key (class-num nil))

  (let* ((class-data-list (elt *all-class-data* 1)))
    (dotimes (i (length class-data-list)) ;; class data list
      (when class-num (setq i class-num)) ;; when class-num is specified
      (dotimes (j (length (elt class-data-list i))) ;; class dat
	(let* ((class-data
		(elt (elt class-data-list i) j))
	       (obj-id
		(cdr (assoc :obj-id class-data)))
	       (manip-id
		(cdr (assoc :manip-id class-data))))
	  (warn "class: ~a/~a  elem: ~a/~a  (obj,manip): (~a,~a) ~%"
		(+ i 1) (length class-data-list)
		(+ j 1) (length (elt class-data-list i))
		obj-id manip-id)
	  (manip :manip-input (elt *manip-list* manip-id) :obj-num obj-id :no-save t)
	  ))
      (when class-num (return-from nil nil)) ;; when class-num is specified
      ;; wait
      (read-line))
    ))

;; util ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; get random force
(defun get-random-force
  ()

  (let* (
	 (fx-list (list -10 -5 -1 0 1 5 10))
	 ;;(fx-list (list -10 -5 -1 0 1 5 10 15 20 25))
	 (fy-list (list 0))
	 (fz-list (list -8 -6 -4 2 -1 0 1 2 4 6 8))
	 ;;(fz-list (list -3 -2 -1 0 1 2 3))
	 )
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
  (&optional (c (send *obj1* :copy-worldcoords)))

  (let* (
	 ;;(rp-angle-thre (/ pi 10))
	 (rp-angle-thre (/ pi 5))
	 (base-vec (float-vector 0 0 1))
	 (trans-base-vec (send c :rotate-vector base-vec))
	 (rp-angle (abs (acos (v. base-vec trans-base-vec)))))
    ;; (warn "~a/~a" rp-angle rp-angle-thre)
    (> rp-angle rp-angle-thre)
    ))

;; test ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; get manip
(defun get-manip-force
  (&key (manip-num nil) (convert-to-eus t))

  (mapcar
   #'(lambda (i)
       (let* ((manip (elt *manip-list* i))
	      (manip-force1 (elt (elt *manip-list* i) 0))
	      (manip-force2 (elt (elt *manip-list* i) (1- (length (elt *manip-list* i))))))
	 (cond (convert-to-eus
		(list
		 (mapcar #'convert-force-from-gazebo-to-eus manip-force1)
		 (mapcar #'convert-force-from-gazebo-to-eus manip-force2)))
	       (t
		(list manip-force1 manip-force2)))
	 ))
   (cond (manip-num (list manip-num))
	 (t (range (length *manip-list*))))
   ))

(defun get-sort-manip-force
  ()

  (mapcar #'(lambda (i) (car (get-manip-force :manip-num i)))
	  (get-sort-manip-id)))

(defun convert-force-from-gazebo-to-eus
  (force)

  (concatenate float-vector
	       (mapcar #'(lambda (x) (if (eps= x 0.0) 0.0 x))
		       (concatenate cons (v- (rotate-vector force pi/2 :y)))))
  )
