#!/usr/bin/env roseus

#-:jsk
(jsk)
#-:rbrain-basic
(rbrain)


(load "package://hrpsys_gazebo_tutorials/euslisp/manip-obj-with-hand-util.l")


(defun usage-plot
  ()

  (warn ";; plot ;;;;;;;;;;;;;;;;;~%")
  (warn "(plot-all-class-data) ;; plot *all-class-data*~%")
  (warn "(plot-all) ;; plot all graph~%")
  (warn "(plot-contact) ;; plot *contact-list*~%")
  (warn "(plot-2dcoord) ;; plot *2dcoord-list*~%")
  )


;; main function ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun plot-all-class-data
  (&key (save nil) (class-num nil))

  (dotimes (i (length (elt *all-class-data* 1)))
    (when class-num (setq i class-num))
    ;; plot
    (plot-contact :data (list (mapcar #'(lambda (x) (cdr (assoc :contact x))) (elt (elt *all-class-data* 1) i))))
    (plot-2dcoord :data (list (mapcar #'(lambda (x) (cdr (assoc :2dcoord x))) (elt (elt *all-class-data* 1) i))))
    (warn "plotting ~d data ~d / ~d. press any key for next plotting. ~%"
	  (length (mapcar #'(lambda (x) (cdr (assoc :contact x))) (elt (elt *all-class-data* 1) i))) (+ i 1) (length (elt *all-class-data* 1)))
    ;; wait
    (read-line)
    ;; save
    (when save
      (save-plot-contact (format nil "fig/contact-graph-~a.png" i))
      (save-plot-2dcoord (format nil "fig/2dcoord-graph-~a.png" i))
      (read-line))
    (when class-num (return-from nil nil))
    ))

;; all ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun plot-all
  (&key (contact-data *contact-list*) (2dcoord-data *2dcoord-list*))

  (plot-contact :data contact-data)
  (plot-2dcoord :data 2dcoord-data))

;; contact ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun plot-contact
  (&key (data *contact-list*) (obj-num nil) (manip-num nil)
	(wait nil) (with-title nil))
  
  ;; init graph
  (unless (boundp '*gnuplot-contact*)
    (setq *gnuplot-contact* (gnuplot)))

  (let* ((plot-file-name "log/tmp-log-for-plot-contact.txt")
	 (plot-log-stream
	  (open plot-file-name :direction :output))
	 (data-num-tmp 0) (data-num nil))
    (dotimes (i (length (elt (elt data 0) 0))) ;; time
      (format plot-log-stream "~a" i)
      (dotimes (j (length data)) ;; manip
	(when manip-num (setq j manip-num)) ;; when manip is specified
	(dotimes (k (length (elt data 0))) ;; obj
	  (when obj-num (setq k obj-num)) ;; when obj is specified
	  (let* ((cont (elt (elt (elt data j) k) i))
		 (cont-num
		  (cond ((equal cont :face) 3)
			((equal cont :line) 2)
			((equal cont :vertex) 1)
			((equal cont :nil) 0)
			((equal cont :invalid) 4))))
	    (format plot-log-stream " ~a" cont-num)) ;; count plot data num
	  (setq data-num-tmp (+ data-num-tmp 1))
	  (when obj-num (return-from nil nil)) ;; when obj is specified
	  )
	(when manip-num (return-from nil nil)) ;; when manip is specified
	)
      (format plot-log-stream "~%")
      (unless data-num (setq data-num data-num-tmp)) ;; set plot data num
      )
    (close plot-log-stream)
    (send *gnuplot-contact* :command "set key outside")
    (send *gnuplot-contact* :command "set yrange [4.5:-0.5]")
    (dotimes (j data-num)
      (cond (with-title (send *gnuplot-contact* :command (format nil "~aplot \"~a\" u 1:~a with line lw 5 title \"~a\""
								 (if (equal j 0) "" "re") plot-file-name (+ j 2) (+ j 1))))
	    (t (send *gnuplot-contact* :command (format nil "~aplot \"~a\" u 1:~a with line lw 5 notitle"
							(if (equal j 0) "" "re") plot-file-name (+ j 2)))))
      (when wait
	(warn "press any key for plotting next line~%")
	(read-line)))
    ))

;; 2dcoord ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun plot-2dcoord 
  (&key (data *2dcoord-list*) (obj-num nil) (manip-num nil)
	(wait nil) (with-title nil))

  (unless (boundp '*gnuplot-2dcoord*)
    (setq *gnuplot-2dcoord* (gnuplot)))

  (let* ((plot-file-name "log/tmp-log-for-plot-2dcoord.txt")
	 (plot-log-stream
	  (open plot-file-name :direction :output))
	 (plot-point-interval 40)
	 ;;(plot-vect-interval 20)
	 (plot-vect-interval 100)
	 (data-num-tmp 0) (data-num nil))
    (dotimes (i (length (elt (elt data 0) 0))) ;; time
      (format plot-log-stream "~a" i)
      (dotimes (j (length data)) ;; manip
	(when manip-num (setq j manip-num)) ;; when manip is specified
	(dotimes (k (length (elt data 0))) ;; obj
	  (when obj-num (setq k obj-num)) ;; when obj is specified
	  (let* ((2d-coord-tmp (elt (elt (elt data j) k) i))
		 (x-tmp (elt (car 2d-coord-tmp) 0))
		 (y-tmp (elt (car 2d-coord-tmp) 1))
		 (theta-tmp (cadr 2d-coord-tmp))
		 (vec-norm-tmp 0.2)
		 (x-tmp2 (* vec-norm-tmp (cos theta-tmp)))
		 (y-tmp2 (* vec-norm-tmp (sin theta-tmp))))
	    (format plot-log-stream " ~a ~a ~a ~a" x-tmp y-tmp x-tmp2 y-tmp2))

	  (setq data-num-tmp (+ data-num-tmp 1)) ;; count plot data num
	  (when obj-num (return-from nil nil)) ;; when obj is specified
	  )
	(when manip-num (return-from nil nil)) ;; when manip is specified
	)
      (format plot-log-stream "~%")
      (unless data-num (setq data-num data-num-tmp)) ;; set plot data num
      )
    (close plot-log-stream)
    (send *gnuplot-2dcoord* :command "set key outside")
    (send *gnuplot-2dcoord* :command "set grid")
    (send *gnuplot-2dcoord* :command "set xrange [-1.5:1.5]")
    (send *gnuplot-2dcoord* :command "set yrange [-1.5:1.5]")
    (dotimes (j data-num)
      (cond (with-title
	     (send *gnuplot-2dcoord* :command (format nil "~aplot \"~a\" u ~a:~a every ~a with line lw 5 title \"~a\""
						      (if (equal j 0) "" "re") plot-file-name (+ (* j 4) 2) (+ (* j 4) 3) plot-point-interval (+ j 1)))
	     (send *gnuplot-2dcoord* :command (format nil "replot \"~a\" u ~a:~a:~a:~a every ~a with vector title \"~a\""
						      plot-file-name (+ (* j 4) 2) (+ (* j 4) 3) (+ (* j 4) 4) (+ (* j 4) 5) plot-vect-interval (+ j 1))))
	    (t
	     (send *gnuplot-2dcoord* :command (format nil "~aplot \"~a\" u ~a:~a every ~a with line lw 5 notitle"
						      (if (equal j 0) "" "re") plot-file-name (+ (* j 4) 2) (+ (* j 4) 3) plot-point-interval))
	     (send *gnuplot-2dcoord* :command (format nil "replot \"~a\" u ~a:~a:~a:~a every ~a with vector notitle"
						      plot-file-name (+ (* j 4) 2) (+ (* j 4) 3) (+ (* j 4) 4) (+ (* j 4) 5) plot-vect-interval))))
      (when wait
	(warn "press any key for plotting next line~%")
	(read-line)))
    ))

;; save ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun save-plot-contact (&optional (filename "fig/contact-graph-tmp.png"))
  (send *gnuplot-contact* :command "set terminal png")
  (send *gnuplot-contact* :command (format nil "set output \"~a\"" filename))
  (send *gnuplot-contact* :command "replot")
  (send *gnuplot-contact* :command "set terminal wxt")
  )

(defun save-plot-2dcoord (&optional (filename "fig/2dcoord-graph-tmp.png"))
  (send *gnuplot-2dcoord* :command "set terminal png")
  (send *gnuplot-2dcoord* :command (format nil "set output \"~a\"" filename))
  (send *gnuplot-2dcoord* :command "replot")
  (send *gnuplot-2dcoord* :command "set terminal wxt")
  )

