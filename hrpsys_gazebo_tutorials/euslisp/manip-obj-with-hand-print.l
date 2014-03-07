#!/usr/bin/env roseus

#-:jsk
(jsk)
#-:rbrain-basic
(rbrain)


(load "package://hrpsys_gazebo_tutorials/euslisp/manip-obj-with-hand-util.l")


(defun usage-print
  ()

  (warn ";; print ;;;;;;;;;;;;;;;;;~%")
  (warn "(usage-analyze)~%")
  (warn "(print-feature-value)~%")
  (warn "(print-manip-eval)~%")
  (warn "(print-obj-info)~%")
  (warn "(print-manip-info)~%")
  (warn "(print-class-info)~%")
  (warn "(print-result)~%")
  )


;; print info ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; feaure-value
(defun print-feature-value
  (&key (fv-data *fv-list*) (eval-data *eval-list*) (class-num nil) (obj-num nil) (manip-num nil))

  (let* (fv-list eval-list)
    (cond (class-num
	   (setq fv-list (list (mapcar #'(lambda (x) (cdr (assoc :fv x))) (elt (elt *all-class-data* 1) class-num))))
	   (setq eval-list (list (mapcar #'(lambda (x) (cdr (assoc :eval x))) (elt (elt *all-class-data* 1) class-num)))))
	  (t
	   (setq fv-list fv-data)
	   (setq eval-list eval-data)))
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

;; eval
(defun print-manip-eval
  (&key (obj-num nil) (manip-num nil))

  ;; print first line
  (warn "-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*~%")
  (warn "~8a|" "mnp\\obj")
  (dotimes (j (length (elt *eval-list* 0))) ;; obj
    (when obj-num (setq j obj-num)) ;; when obj is specified
    (warn "    ~8a" j)
    (when obj-num (return-from nil nil)) ;; when obj is specified
    )
  (warn "    ~8a" "ave")
  (warn "~%")
  (warn "--------------------------------------------------------------------~%")
  ;; print data
  (dotimes (i (length *eval-list*)) ;; manip
    (when manip-num (setq i manip-num)) ;; when manip is specified
    (warn "~8a|" i)
    (dotimes (j (length (elt *eval-list* 0))) ;; obj
      (when obj-num (setq j obj-num)) ;; when obj is specified
      (warn "    ~8a" (elt (elt *eval-list* i) j))
      (when obj-num (return-from nil nil)) ;; when obj is specified
      )
    (let* (manip-eval-list manip-eval)
      (setq manip-eval-list (elt *eval-list* i))
      (setq manip-eval (/ (reduce #'+ manip-eval-list) (length manip-eval-list)))
      (warn "    ~8a" manip-eval))
    (warn "~%")
    (when manip-num (return-from nil nil)) ;; when manip is specified
    )
  ;; print obj ave
  (warn "~8a|" "ave")
  (dotimes (j (length (elt *eval-list* 0))) ;; obj
    (when obj-num (setq j obj-num)) ;; when obj is specified
    (let* (obj-eval-list obj-eval)
      (setq obj-eval-list (mapcar #'(lambda (x) (elt x j)) *eval-list*))
      (setq obj-eval (/ (reduce #'+ obj-eval-list) (length obj-eval-list)))
      (warn "    ~8a" obj-eval))
    (when obj-num (return-from nil nil)) ;; when obj is specified
    )
  (warn "~%")
  (warn "-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*~%")
  )

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
  (&key (manip-num nil) (manip-eval-list-flag nil))

  (warn "-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*~%")

  (cond (manip-eval-list-flag
	 (warn " ~10a  ~80a  ~10a  ~20a~%"
	       "id\\value" "force" "eval" "eval-list"))
	(t
	 (warn " ~10a  ~80a  ~10a~%"
	       "id\\value" "force" "eval")))
  (dotimes (i (length *eval-list*))
    (let* (manip-eval manip-eval-list)
      (when manip-num (setq i manip-num)) ;; when manip is specified
      (setq manip-eval-list (elt *eval-list* i))
      (setq manip-eval (/ (reduce #'+ manip-eval-list) (length manip-eval-list)))
      (cond (manip-eval-list-flag
	     (warn " ~10a  ~80a  ~10a  ~20a~%"
		   i (list (elt (elt *manip-list* i) 0) (elt (elt *manip-list* i) (1- (length *manip-list*)))) manip-eval manip-eval-list))
	    (t
	     (warn " ~10a  ~80a  ~10a~%"
		   i (list (elt (elt *manip-list* i) 0) (elt (elt *manip-list* i) (1- (length *manip-list*)))) manip-eval)))
      (when manip-num (return-from nil nil)) ;; when manip is specified
      ))
  (warn "-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*~%")
  )

;; class
(defun print-class-info
  (&key (class-num nil) (class-inside-eval-list-flag nil))

  (warn "-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*~%")
  (warn " ~10a  ~10a  ~10a  ~20a  ~20a~%"
	"id\\value" "elem" "eval" "fv" "eval-list")
  (dotimes (i (length (elt *all-class-data* 1)))
    (let* (class-data
	   class-elem-num class-eval class-fv class-inside-eval-list)
      (when class-num ;; when class-num is specified, overwrite with specified num
	(setq i class-num))
      (setq class-data (elt (elt *all-class-data* 1) i))
      (setq class-elem-num (length class-data))
      (setq class-eval (elt *class-eval* i))
      (setq class-fv (elt *class-fv* i))
      (setq class-inside-eval-list
	    (mapcar #'(lambda (x) (cdr (assoc :eval x))) class-data))   
      (warn " ~10a  ~10a  ~10a  ~20a  ~20a~%"
	    i class-elem-num class-eval class-fv
	    (cond (class-inside-eval-list-flag
		   class-inside-eval-list)
		  (t
		   nil)))
      (when class-num (return-from nil nil))))
  (warn "-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*~%")
  )

;; result
(defun print-result
  (&key (obj-num nil) (manip-num nil))

  ;; print first line
  (warn "-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*~%")
  (warn "~10a|" "mnp\\obj")
  (dotimes (j (length (elt *class-list* 0))) ;; obj
    (when obj-num (setq j obj-num)) ;; when obj is specified
    (warn "    ~10a" j)
    (when obj-num (return-from nil nil)) ;; when obj is specified
    )
  (warn "    ~10a" "ave")
  (warn "~%")
  (warn "--------------------------------------------------------------------~%")
  ;; print data
  (dotimes (i (length *class-list*)) ;; manip
    (when manip-num (setq i manip-num)) ;; when manip is specified
    (warn "~10a|" i)
    (dotimes (j (length (elt *class-list* 0))) ;; obj
      (when obj-num (setq j obj-num)) ;; when obj is specified
      (warn "    ~2a(~6,2f)"
	    (elt (elt *class-list* i) j)
	    (elt *class-eval* (elt (elt *class-list* i) j)))
      (when obj-num (return-from nil nil)) ;; when obj is specified
      )
    (let* (manip-eval-list manip-eval)
      (setq manip-eval-list (mapcar #'(lambda (x) (elt *class-eval* x)) (elt *class-list* i)))
      (setq manip-eval (/ (reduce #'+ manip-eval-list) (length manip-eval-list)))
      (warn "    ~2a[~6,2f]" "" manip-eval))
    (warn "~%")
    (when manip-num (return-from nil nil)) ;; when manip is specified
    )
  ;; print obj ave
  (warn "~10a|" "ave")
  (dotimes (j (length (elt *class-list* 0))) ;; obj
    (when obj-num (setq j obj-num)) ;; when obj is specified
    (let* (obj-eval-list obj-eval)
      (setq obj-eval-list (mapcar #'(lambda (x) (elt *class-eval* x)) (mapcar #'(lambda (x) (elt x j)) *class-list*)))
      (setq obj-eval (/ (reduce #'+ obj-eval-list) (length obj-eval-list)))
      (warn "    ~2a[~6,2f]" "" obj-eval))
    (when obj-num (return-from nil nil)) ;; when obj is specified
    )
  (warn "~%")
  (warn "-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*~%")  
  )


;; print util ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun print-class-list
  ()

  (warn "-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*~%")  
  (warn "-*- train -*-~%")
  (print-class-list-train)
  (warn "-*- test -*-~%")
  (print-class-list-test)
  (warn "-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*~%")  
  )

(defun print-class-list-train
  ()

  (dolist (class-list-data (get-class-list))
    (warn "~a~%" class-list-data)))

(defun print-class-list-test
  ()

  (warn "~a~%" (get-nearest-class))
  )
