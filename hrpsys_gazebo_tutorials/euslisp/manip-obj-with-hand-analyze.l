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
	       (d-vel-thre 0.1)
	       (d-vel-new (if (> d-vel d-vel-thre) d-vel 0))
	       (xy-unit-vel (if (< d-vel d-vel-thre) (float-vector 0 0) (normalize-vector xy-vel)))
	       (yaw-crnt (cadr (elt 2dcoord i)))
	       (yaw-prev (cadr (elt 2dcoord (- i 1))))
	       (yaw-vel (abs (/ (- yaw-crnt yaw-prev) *manip-dt*)))
	       (yaw-vel-thre 0.1)
	       (yaw-vel-new (if (> yaw-vel yaw-vel-thre) yaw-vel 0))
	       (yaw-unit-vel (float-vector (cos yaw-crnt) (sin yaw-crnt)))
	       (cor-xy-yaw (abs (v. xy-unit-vel yaw-unit-vel)))
	       (float-max 500)
	       (fall-down-max 500)
	       )

	  ;; (print (list yaw-vel d-vel xy-unit-vel yaw-unit-vel)) ;; for debug
	  (cond ((equal cont-num -1) ;; when falling down
		 (setq fv-fall-down (min (+ fv-fall-down 10) fall-down-max)))
		((equal cont-num 0) ;; when floating
		 ;; each contact state
		 (setq fv-float (min (+ fv-float 10) float-max)))
		((>= cont-num 0)
		 ;; each contact state
		 (setf (elt fv-yaw-vel cont-num) (+ (elt fv-yaw-vel cont-num) yaw-vel-new))
		 (setf (elt fv-d-vel cont-num) (+ (elt fv-d-vel cont-num) d-vel-new))
		 (setf (elt fv-cor-xy-yaw cont-num) (+ (elt fv-cor-xy-yaw cont-num) cor-xy-yaw))
		 ;; sum
		 (setf (elt fv-yaw-vel 4) (+ (elt fv-yaw-vel 4) yaw-vel))
		 (setf (elt fv-d-vel 4) (+ (elt fv-d-vel 4) d-vel))
		 (setf (elt fv-cor-xy-yaw 4) (+ (elt fv-cor-xy-yaw 4) cor-xy-yaw))))
	  )))
    (setq fv-eval (calc-manip-eval (car (last contact)) (car (last 2dcoord))))

    ;; square root instead of normalize
    (dotimes (cont-num 4)
      (setf (elt fv-yaw-vel cont-num) (sqrt (elt fv-yaw-vel cont-num)))
      (setf (elt fv-d-vel cont-num) (sqrt (elt fv-d-vel cont-num)))
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
  (&key (output t))

  (setq *fv-list* (make-list (length *contact-list*)))
  (setq *eval-list* (make-list (length *contact-list*)))
  (dotimes (i (length *contact-list*)) ;; manip
    (dotimes (j (length (elt *contact-list* 0))) ;; obj
      (let* ((contact (elt (elt *contact-list* i) j))
	     (2dcoord (elt (elt *2dcoord-list* i) j))
	     (fv-eval-list (calc-feature-value-one contact 2dcoord)))
	(push-back (elt fv-eval-list 0) (elt *fv-list* i))
	(push-back (elt fv-eval-list 1) (elt *eval-list* i)))))
  (when output
    (print-feature-value))
  )

(defun print-feature-value
  (&key (class-num nil) (obj-num nil) (manip-num nil))

  (let* (fv-list eval-list)
    (cond (class-num
	   (setq fv-list (mapcar #'(lambda (x) (cdr (assoc :fv x))) (elt (elt *all-class-data* 1) class-num)))
	   (setq eval-list (mapcar #'(lambda (x) (cdr (assoc :eval x))) (elt (elt *all-class-data* 1) class-num))))
	  (t
	   (setq fv-list *fv-list*)
	   (setq eval-list *eval-list*)))
    (dotimes (j (length fv-list)) ;; manip
      (when manip-num (setq j manip-num)) ;; when manip is specified
      (dotimes (k (length (elt fv-list 0))) ;; obj
	(when obj-num (setq k obj-num)) ;; when obj is specified
	(warn "--------------------------------------------------------------------~%")
	(warn " ~10a  ~10a  ~10a  ~10a  ~10a~%"
	      "cont\\fv" "fv-yaw" "fv-d" "fv-cor" "fall/float/eval")
	(warn "--------------------------------------------------------------------~%")
	(let* ((fv (elt (elt fv-list j) k))
	       (eval (elt (elt eval-list j) k)))
	  (dotimes (i (/ (1- (length fv)) 3))
	    (warn " ~10a  ~10a  ~10a  ~10a  ~10a~%"
		  (case i (0 :nil) (1 :vertex) (2 :line) (3 :face) (4 :sum))
		  (elt fv i)
		  (elt fv (+ i (/ (1- (length fv)) 3)))
		  (elt fv (+ i (* (/ (1- (length fv)) 3) 2)))
		  (cond ((< (+ i (* (/ (1- (length fv)) 3) 3)) (length fv))
			 (elt fv (+ i (* (/ (1- (length fv)) 3) 3))))
			((equal i (1- (/ (1- (length fv)) 3)))
			 eval)
			(t
			 "--")))))
	(when obj-num (return-from nil nil)) ;; when obj is specified
	)
      (when manip-num (return-from nil nil)) ;; when manip is specified
      )
    (warn "--------------------------------------------------------------------~%")))

;; classify feature value ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun classify-feature-value
  (&key (class-num 3))

  ;; (load "/home/murooka/prog/jsk-ros-pkg-unreleased.bak.0218/jsk_rosmake_libs/euslib/jsk/lmeds.l")
  (setq *all-orig-data* nil)
  (dotimes (i (length *fv-list*))
    (push-back (list (cons :manip (elt *manip-list* i))
		     (cons :contact (elt *contact-list* i))
		     (cons :2dcoord (elt *2dcoord-list* i))
		     (cons :fv (elt *fv-list* i))
		     (cons :eval (elt *eval-list* i)))
	       *all-orig-data*))
  (setq *all-class-data* (k-means *all-orig-data* class-num :key #'(lambda (x) (cdr (assoc :fv x))) :debug nil))
  ;; sort result depending on element number
  (unless (equal *all-class-data* :error)
    (setf (elt *all-class-data* 1)
	  (sort (elt *all-class-data* 1) #'(lambda (x y) (<= (length x) (length y))))))
  nil)

;; evaluate ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; calc eval value
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

(defun calc-class-eval
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
    class-eval-list
    ))

;; print info ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; obj
(defun print-obj-info
  (&key (obj-num nil))

  (warn "-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*~%")
  (warn " ~10a  ~10a  ~20a~%"
	"id\\value" "eval" "eval-list")
  (dotimes (i (length (elt *eval-list* 0)))
    (let* (obj-eval obj-eval-list)
      (when obj-num (setq i obj-num)) ;; when obj is specified
      (setq obj-eval-list (mapcar #'(lambda (x) (elt x i)) *eval-list*))
      (setq obj-eval (/ (reduce #'+ obj-eval-list) (length obj-eval-list)))
      (warn " ~10a  ~10a  ~20a~%"
	    i obj-eval obj-eval-list)
      (when obj-num (return-from nil nil)) ;; when obj is specified
      ))
  (warn "-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*~%")
  )

;; manip
(defun print-manip-info
  (&key (manip-num nil))

  (warn "-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*~%")
  (warn " ~10a  ~10a  ~20a~%"
	"id\\value" "eval" "eval-list")
  (dotimes (i (length *eval-list*))
    (let* (manip-eval manip-eval-list)
      (when manip-num (setq i manip-num)) ;; when manip is specified
      (setq manip-eval-list (elt *eval-list* i))
      (setq manip-eval (/ (reduce #'+ manip-eval-list) (length manip-eval-list)))
      (warn " ~10a  ~10a  ~20a~%"
	    i manip-eval manip-eval-list)
      (when manip-num (return-from nil nil)) ;; when manip is specified
      ))
  (warn "-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*~%")
  )

;; class
(defun print-class-info
  (&key (class-num nil) (class-inside-eval-list-flag nil))

  (warn "-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*~%")
  (warn " ~10a  ~10a  ~10a  ~20a~%"
	"id\\value" "elem" "eval" "eval-list")
  (dotimes (i (length (elt *all-class-data* 1)))
    (let* (class-data
	   class-elem-num class-eval class-inside-eval-list)
      (when class-num ;; when class-num is specified, overwrite with specified num
	(setq i class-num))
      (setq class-data (elt (elt *all-class-data* 1) i))
      (setq class-elem-num (length class-data))
      (setq class-eval (calc-class-eval :class-num i))
      (setq class-inside-eval-list
	    (mapcar #'(lambda (x) (cdr (assoc :eval x))) class-data))   
      (warn " ~10a  ~10a  ~10a  ~10a~%"
	    i class-elem-num class-eval
	    (cond (class-inside-eval-list-flag
		   class-inside-eval-list)
		  (t
		   nil)))
      (when class-num (return-from nil nil))))
  (warn "-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*~%")
  )
