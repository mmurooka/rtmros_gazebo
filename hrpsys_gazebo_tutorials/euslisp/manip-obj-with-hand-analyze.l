#!/usr/bin/env roseus

#-:jsk
(jsk)
#-:rbrain-basic
(rbrain)


(load "package://hrpsys_gazebo_tutorials/euslisp/manip-obj-with-hand-util.l")


(defun usage-analyze
  ()

  (warn ";; analyze ;;;;;;;;;;;;;;;;;~%")
  (warn "(analyze)~%")
  (warn "(calc-feature-value)~%")
  (warn "(classify-feature-value)~%")
  )


;; main function ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun analyze
  (&key (class-num 3))

  ;; calc feature value
  (calc-feature-value :output nil)
  ;; classify
  (classify-feature-value :class-num class-num)
  (when (equal *all-class-data* :error)
    (return-from analyze nil nil))
  ;; calc class eval
  (calc-class-fv-eval)
  ;; update class list
  (update-class-list)
  t)

;; calc feature value ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; calc
(defun calc-feature-value-one
  (&optional (contact (car *contact-list*)) (2dcoord (car *2dcoord-list*)))

  (let* ((fv-yaw-vel (make-list 5 :initial-element 0)) ;; yaw velocity
	 (fv-d-vel (make-list 5 :initial-element 0)) ;; 2d velocity
	 (fv-cor-xy-yaw (make-list 5 :initial-element 0)) ;; correlation between yaw and 2d
	 (fv-fall-down 0) ;; falling down
	 (fv-float 0) ;; floating
	 (fv-eval 0) ;; eval
	 )
    (dotimes (i (length contact))
      (unless (equal i 0)
	(let* ((cont (elt contact i))
	       (cont-num
		(cond ((equal cont :face) 3)
		      ((equal cont :line) 2)
		      ((equal cont :vertex) 1)
		      ((equal cont :nil) 0)
		      ((equal cont :invalid) -1)))
	       (x-crnt (elt (car (elt 2dcoord i)) 0))
	       (x-prev (elt (car (elt 2dcoord (- i 1))) 0))
	       (y-crnt (elt (car (elt 2dcoord i)) 1))
	       (y-prev (elt (car (elt 2dcoord (- i 1))) 1))
	       (xy-vel (float-vector (- x-crnt x-prev) (- y-crnt y-prev)))
	       (d-vel (abs (/ (norm xy-vel) *manip-dt*)))
	       (d-vel-thre 0.01)
	       (d-vel-new (if (> d-vel d-vel-thre) d-vel 0))
	       (xy-unit-vel (if (< d-vel d-vel-thre) (float-vector 0 0) (normalize-vector xy-vel)))
	       (yaw-crnt (cadr (elt 2dcoord i)))
	       (yaw-prev (cadr (elt 2dcoord (- i 1))))
	       (yaw-vel (abs (/ (- yaw-crnt yaw-prev) *manip-dt*)))
	       (yaw-vel-thre 0.01)
	       (yaw-vel-new (if (> yaw-vel yaw-vel-thre) yaw-vel 0))
	       (yaw-unit-vel (float-vector (cos yaw-crnt) (sin yaw-crnt)))
	       (cor-xy-yaw (abs (v. xy-unit-vel yaw-unit-vel)))
	       (float-max 500)
	       (fall-down-max 500)
	       )

	  ;;(print (list yaw-crnt yaw-unit-vel xy-unit-vel cor-xy-yaw)) ;; for debug
	  (cond ((equal cont-num -1) ;; when falling down
		 (setq fv-fall-down (min (+ fv-fall-down 10) fall-down-max))
		 ;; each contact state
		 (setf (elt fv-yaw-vel 0) (max (+ (elt fv-yaw-vel 0) -1) (- fall-down-max)))
		 (setf (elt fv-d-vel 0) (max (+ (elt fv-d-vel 0) -1) (- fall-down-max)))
		 (setf (elt fv-cor-xy-yaw 0) (max (+ (elt fv-cor-xy-yaw 0) -1) (- fall-down-max)))
		 )
		((equal cont-num 0) ;; when floating
		 (setq fv-float (min (+ fv-float 10) float-max))
		 ;; each contact state
		 (setf (elt fv-yaw-vel 0) (min (+ (elt fv-yaw-vel 0) 1) float-max))
		 (setf (elt fv-d-vel 0) (min (+ (elt fv-d-vel 0) 1) float-max))
		 (setf (elt fv-cor-xy-yaw 0) (min (+ (elt fv-cor-xy-yaw 0) 1) float-max))
		 )
		((>= cont-num 0)
		 ;; each contact state
		 (setf (elt fv-yaw-vel cont-num) (+ (elt fv-yaw-vel cont-num) yaw-vel-new))
		 (setf (elt fv-d-vel cont-num) (+ (elt fv-d-vel cont-num) d-vel-new))
		 (setf (elt fv-cor-xy-yaw cont-num) (+ (elt fv-cor-xy-yaw cont-num) cor-xy-yaw))
		 ;; sum
		 (setf (elt fv-yaw-vel 4) (+ (elt fv-yaw-vel 4) yaw-vel))
		 (setf (elt fv-d-vel 4) (+ (elt fv-d-vel 4) d-vel))
		 (setf (elt fv-cor-xy-yaw 4) (+ (elt fv-cor-xy-yaw 4) cor-xy-yaw))
		 ))
	  )))
    (setq fv-eval (calc-manip-eval (car (last contact)) (car (last 2dcoord))))

    ;; square root instead of normalize
    (dotimes (cont-num 4)
      ;;(setf (elt fv-yaw-vel cont-num) (sqrt-with-sgn (elt fv-yaw-vel cont-num)))
      (setf (elt fv-d-vel cont-num) (sqrt-with-sgn (elt fv-d-vel cont-num)))
      (setf (elt fv-cor-xy-yaw cont-num) (elt fv-cor-xy-yaw cont-num))
      ;;(setq fv-float (sqrt fv-float))
      ;;(setq fv-fall-down (sqrt fv-fall-down))
      )

    (list
     (concatenate float-vector (append (subseq fv-yaw-vel 0 4) (subseq fv-d-vel  0 4) (subseq fv-cor-xy-yaw 0 4) (list fv-fall-down fv-float))) ;; exclude sum
     fv-eval)
    ;;(concatenate float-vector (append fv-yaw-vel fv-d-vel (list 0 0) (subseq fv-cor-xy-yaw 2))) ;; include sum
    ))

(defun calc-feature-value
  (&key (output t) (contact-data *contact-list*) (2dcoord-data *2dcoord-list*) (no-save nil))

  (cond ((equal no-save nil)
	 (setq *fv-list* (make-list (length contact-data)))
	 (setq *eval-list* (make-list (length contact-data))))
	((equal no-save :test)
	 (setq *fv-list-test* (make-list 1))
	 (setq *eval-list-test* (make-list 1))))
  (dotimes (i (length contact-data)) ;; manip
    (dotimes (j (length (elt contact-data 0))) ;; obj
      (let* ((contact (elt (elt contact-data i) j))
	     (2dcoord (elt (elt 2dcoord-data i) j))
	     (fv-eval-list (calc-feature-value-one contact 2dcoord)))
	(cond ((equal no-save nil)
	       (push-back (elt fv-eval-list 0) (elt *fv-list* i))
	       (push-back (elt fv-eval-list 1) (elt *eval-list* i)))
	      ((equal no-save :test)
 	       (push-back (elt fv-eval-list 0) (elt *fv-list-test* 0))
	       (push-back (elt fv-eval-list 1) (elt *eval-list-test* 0)))
	      ))))
  (when output
    (cond ((equal no-save nil)
	   (print-feature-value))
	  ((equal no-save :test)
	   (print-feature-value :fv-data *fv-list-test* :eval-data *eval-list-test*))))
  )

;; classify feature value ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun classify-feature-value
  (&key (class-num 3))

  ;; set original data
  (setq *all-orig-data* nil)
  (dotimes (i (length *fv-list*)) ;; manip
    (dotimes (j (length (elt *fv-list* 0))) ;; obj
      (push-back (list (cons :manip (elt *manip-list* i))
		       (cons :contact (elt (elt *contact-list* i) j))
		       (cons :2dcoord (elt (elt *2dcoord-list* i) j))
		       (cons :eval (elt (elt *eval-list* i) j))
		       (cons :fv (elt (elt *fv-list* i) j))
		       (cons :obj-id j)
		       (cons :manip-id i)
		       )
		 *all-orig-data*)))
  ;; classify
  ;; (load "/home/murooka/prog/jsk-ros-pkg-unreleased.bak.0218/jsk_rosmake_libs/euslib/jsk/lmeds.l")
  (dotimes (i 10)
    (setq *all-class-data* (k-means *all-orig-data* class-num :key #'(lambda (x) (cdr (assoc :fv x))) :debug nil))
    (when (not (equal *all-class-data* :error))
      (return-from nil nil)))

  ;; sort result depending on element number
  (unless (equal *all-class-data* :error)
    (let* ((class-fv-class-data-list nil)
	   (class-fv-list
	    (mapcar #'(lambda (i) (matrix-row (elt *all-class-data* 0) i)) (range (length (elt *all-class-data* 1)))))
	   (class-data-list (elt *all-class-data* 1)))
      ;; set data to new variable for sorting
      (setq class-fv-class-data-list (mapcar #'(lambda (x y) (cons x y)) class-fv-list class-data-list))
      ;; sort
      (setq class-fv-class-data-list
	    (sort class-fv-class-data-list #'(lambda (x y) (<= (length (cdr x)) (length (cdr y))))))
      ;; set sort data to original variable
      (setf (elt *all-class-data* 0) nil)
      (setf (elt *all-class-data* 1) nil)
      (dolist (class-fv-class-data class-fv-class-data-list)
	(push-back (car class-fv-class-data) (elt *all-class-data* 0))
	(push-back (cdr class-fv-class-data) (elt *all-class-data* 1))
	)))
  nil)

;; evaluate ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; calc
(defun calc-manip-eval
  (contact 2dcoord)

  (let* ((dist (norm (subseq (car 2dcoord) 0 2)))
	 (dist-score 0)
	 (fall-down-score 0)
	 (float-score 0))
    (setq dist-score (* dist 100))
    (cond ((equal contact :invalid)
	   (setq fall-down-score -100))
	  ((equal contact :nil)
	   (setq float-score 100)))
    (+ dist-score fall-down-score float-score)
    ))

(defun calc-class-fv-eval
  (&key (class-num nil))

  (let* (class-eval-list)
    (dolist (class-data (elt *all-class-data* 1))
      (when class-num ;; when class-num is specified, overwrite with specified class
	(setq class-data (elt (elt *all-class-data* 1) class-num)))
      (let* (class-inside-eval-list class-eval)
	(setq class-inside-eval-list (mapcar #'(lambda (x) (cdr (assoc :eval x))) class-data))
	(setq class-eval (/ (reduce #'+ class-inside-eval-list) (length class-inside-eval-list)))
	(cond (class-num
	       (setq class-eval-list class-eval))
	      (t
	       (push-back class-eval class-eval-list))))
      (when class-num (return-from nil nil)))
    (unless class-num
      (setq *class-eval* class-eval-list)
      (setq *class-fv* (elt *all-class-data* 0)))
    class-eval-list
    ))

;; update class ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun update-class-list
  ()

  (setq *class-list* (make-list (length *eval-list*))) ;; manip length
  (dotimes (i (length *class-list*))
    (setf (elt *class-list* i) (make-list (length (elt *eval-list* 0))))) ;; obj length
  (let* ((class-data-list (elt *all-class-data* 1))
	 (class-num (length class-data-list)))
    (dotimes (i class-num)
      (dolist (class-data (elt class-data-list i))
	(let* ((obj-id (cdr (assoc :obj-id class-data)))
	       (manip-id (cdr (assoc :manip-id class-data))))
	  (setf (elt (elt *class-list* manip-id) obj-id) i)
	  )))
    ))

;; search class ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun get-nearest-class
  ()

  (mapcar #'get-nearest-class-one (car *fv-list-test*)))

(defun get-nearest-class-one
  (fv)

  (let* ((dist-list
	  (mapcar #'(lambda (class-fv) (norm (v- fv class-fv))) *class-fv*))
	 (min-dist (apply #'min dist-list))
	 (min-id (position min-dist dist-list)))
    min-id))
