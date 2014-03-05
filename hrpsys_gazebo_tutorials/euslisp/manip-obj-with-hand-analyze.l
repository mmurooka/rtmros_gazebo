#!/usr/bin/env roseus

#-:jsk
(jsk)
#-:rbrain-basic
(rbrain)


(load "package://hrpsys_gazebo_tutorials/euslisp/manip-obj-with-hand-util.l")


(defun usage-analyze
  ()

  (warn ";; analize ;;;;;;;;;;;;;;;;;~%")
  (warn "(analyze)~%")
  (warn "(calc-feature-value)~%")
  (warn "(classify-feature-value)~%")
  )


;; main function ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun analyze
  (&key (class-num 3))

  (calc-feature-value :output nil)
  (classify-feature-value :class-num class-num)
  (not (equal *all-class-data* :error)))

;; calc feature value ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun calc-feature-value-one
  (&optional (cont-st (car *contact-list*)) (2d-coord (car *2dcoord-list*)))

  (let* ((fv-yaw-vel (make-list 5 :initial-element 0)) ;; yaw velocity
	 (fv-d-vel (make-list 5 :initial-element 0)) ;; 2d velocity
	 (fv-cor-xy-yaw (make-list 5 :initial-element 0))) ;; correlation between yaw and 2d
    (dotimes (i (length cont-st))
      (unless (equal i 0)
	(let* ((cont (elt cont-st i))
	       (cont-num
		(cond ((equal cont :face) 3)
		      ((equal cont :line) 2)
		      ((equal cont :vertex) 1)
		      ((equal cont :nil) 0)
		      ((equal cont :invalid) 0)))
	       (x-crnt (elt (car (elt 2d-coord i)) 0))
	       (x-prev (elt (car (elt 2d-coord (- i 1))) 0))
	       (y-crnt (elt (car (elt 2d-coord i)) 1))
	       (y-prev (elt (car (elt 2d-coord (- i 1))) 1))
	       (xy-vel (float-vector (- x-crnt x-prev) (- y-crnt y-prev)))
	       (d-vel (abs (/ (norm xy-vel) *manip-dt*)))
	       (d-vel-thre 0.1)
	       (xy-unit-vel (if (< d-vel d-vel-thre) (float-vector 0 0) (normalize-vector xy-vel)))
	       (yaw-crnt (cadr (elt 2d-coord i)))
	       (yaw-prev (cadr (elt 2d-coord (- i 1))))
	       (yaw-vel (abs (/ (- yaw-crnt yaw-prev) *manip-dt*)))
	       (yaw-unit-vel (float-vector (cos yaw-crnt) (sin yaw-crnt)))
	       (cor-xy-yaw (abs (v. xy-unit-vel yaw-unit-vel))))

	  ;; (print (list yaw-vel d-vel xy-unit-vel yaw-unit-vel)) ;; for debug
	  (cond ((equal cont-num 0)
		 ;; each contact state
		 (setf (elt fv-yaw-vel cont-num) (+ (elt fv-yaw-vel cont-num) 1))
		 (setf (elt fv-d-vel cont-num) (+ (elt fv-d-vel cont-num) 1))
		 (setf (elt fv-cor-xy-yaw cont-num) (+ (elt fv-cor-xy-yaw cont-num) 1))
		 ;; sum
		 (setf (elt fv-yaw-vel 4) (+ (elt fv-yaw-vel 4) 1))
		 (setf (elt fv-d-vel 4) (+ (elt fv-d-vel 4) 1))
		 (setf (elt fv-cor-xy-yaw 4) (+ (elt fv-cor-xy-yaw 4) 1)))
		(t
		 ;; each contact state
		 (setf (elt fv-yaw-vel cont-num) (+ (elt fv-yaw-vel cont-num) yaw-vel))
		 (setf (elt fv-d-vel cont-num) (+ (elt fv-d-vel cont-num) d-vel))
		 (setf (elt fv-cor-xy-yaw cont-num) (+ (elt fv-cor-xy-yaw cont-num) cor-xy-yaw))
		 ;; sum
		 (setf (elt fv-yaw-vel 4) (+ (elt fv-yaw-vel 4) yaw-vel))
		 (setf (elt fv-d-vel 4) (+ (elt fv-d-vel 4) d-vel))
		 (setf (elt fv-cor-xy-yaw 4) (+ (elt fv-cor-xy-yaw 4) cor-xy-yaw))))
	  )))

    ;; normalize
    (dotimes (cont-num 4)
      (unless (eps= (elt fv-yaw-vel 4) 0.0)
    	(setf (elt fv-yaw-vel cont-num) (/ (elt fv-yaw-vel cont-num) (elt fv-yaw-vel 4))))
      (unless (eps= (elt fv-d-vel 4) 0.0)
    	(setf (elt fv-d-vel cont-num) (/ (elt fv-d-vel cont-num) (elt fv-d-vel 4))))
      (unless (eps= (elt fv-cor-xy-yaw 4) 0.0)
    	(setf (elt fv-cor-xy-yaw cont-num) (/ (elt fv-cor-xy-yaw cont-num) (/ (length cont-st) 2.0)))))

    ;; (print (list fv-yaw-vel fv-d-vel fv-cor-xy-yaw)) ;; for debug

    (concatenate float-vector (append (subseq fv-yaw-vel 0 4) (subseq fv-d-vel  0 4) (subseq fv-cor-xy-yaw 0 4)))
    ;;(concatenate float-vector (append (subseq fv-yaw-vel 0 4) (subseq fv-d-vel  0 4) (list 0 0) (subseq fv-cor-xy-yaw 2 4)))
    ;;(concatenate float-vector (append fv-yaw-vel fv-d-vel (list 0 0) (subseq fv-cor-xy-yaw 2))) ;; include sum
    ))

(defun calc-feature-value
  (&key (output t))

  (setq *fv-list* nil)
  (dotimes (i (length *contact-list*))
    (let*
	((cont-st (elt *contact-list* i))
	 (2d-coord (elt *2dcoord-list* i)))
      (push-back (calc-feature-value-one cont-st 2d-coord) *fv-list*)))
  (when output
    (print-feature-value))
  )

(defun print-feature-value
  ()

  (dolist (fv *fv-list*)
    (warn "----------------------------------------------------------------~%")
    (warn " ~10a  ~16a  ~16a  ~16a~%"
	  "cont\\fv" "fv-yaw-vel" "fv-d-vel" "fv-cor-xy-yaw")
    (warn "----------------------------------------------------------------~%")
    (dotimes (i (/ (length  fv) 3))
      (warn " ~10a  ~16a  ~16a  ~16a~%"
	    (case i (0 :nil) (1 :vertex) (2 :line) (3 :face) (4 :sum)) (elt fv i) (elt fv (+ i (/ (length  fv) 3))) (elt fv (+ i (* (/ (length  fv) 3) 2))))))
  (warn "----------------------------------------------------------------~%"))

;; classify feature value ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun classify-feature-value
  (&key (class-num 3))

  ;; (load "/home/murooka/prog/jsk-ros-pkg-unreleased.bak.0218/jsk_rosmake_libs/euslib/jsk/lmeds.l")
  (setq *all-orig-data* nil)
  (dotimes (i (length *fv-list*))
    (push-back (list (cons :manip (elt *manip-list* i)) (cons :contact (elt *contact-list* i)) (cons :2dcoord (elt *2dcoord-list* i)) (cons :fv (elt *fv-list* i)))
	       *all-orig-data*))
  (setq *all-class-data* (k-means *all-orig-data* class-num :key #'(lambda (x) (cdr (assoc :fv x))) :debug nil))
  nil)
