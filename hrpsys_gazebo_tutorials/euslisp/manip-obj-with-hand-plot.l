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
  (&key (save t))

  (dotimes (i (length (elt *all-class-data* 1)))
    ;; plot
    (plot-contact :data (mapcar #'(lambda (x) (cdr (assoc :contact x))) (elt (elt *all-class-data* 1) i)))
    (plot-2dcoord :data (mapcar #'(lambda (x) (cdr (assoc :2dcoord x))) (elt (elt *all-class-data* 1) i)))
    (warn "plotting ~d data ~d / ~d. press any key for next plotting. ~%"
	  (length (mapcar #'(lambda (x) (cdr (assoc :contact x))) (elt (elt *all-class-data* 1) i))) (+ i 1) (length (elt *all-class-data* 1)))
    ;; wait
    (read-line)
    ;; save
    (when save
      (save-plot-contact (format nil "fig/contact-graph-~a.png" i))
      (save-plot-2dcoord (format nil "fig/2dcoord-graph-~a.png" i))
      (read-line))
    ))

;; all ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun plot-all
  ()

  (plot-contact)
  (plot-2dcoord))

;; contact ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun plot-contact
  (&key (data *contact-list*) (wait nil) (with-title nil))
  
  ;; init graph
  (unless (boundp '*gnuplot-contact*)
    (setq *gnuplot-contact* (gnuplot)))

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
		      ((equal cont :nil) 0)
		      ((equal cont :invalid) 4))))
	  (format plot-log-stream " ~a" cont-num)))
      (format plot-log-stream "~%")
      )
    (close plot-log-stream)
    (send *gnuplot-contact* :command "set key outside")
    (send *gnuplot-contact* :command "set yrange [4.5:-0.5]")
    (cond (with-title (send *gnuplot-contact* :command (format nil "plot \"~a\" u 1:2 with line lw 5 title \"1\"" plot-file-name)))
	  (t (send *gnuplot-contact* :command (format nil "plot \"~a\" u 1:2 with line lw 5 notitle" plot-file-name))))
    (when (> (length data) 1)
      (dotimes (j (length data))
	(unless (equal j 0)
	  (when wait
	    (warn "press any key for plotting next line~%")
	    (read-line))
	  (cond (with-title (send *gnuplot-contact* :command (format nil "replot \"~a\" u 1:~a with line lw 5 title \"~a\"" plot-file-name (+ j 2) (+ j 1))))
		(t (send *gnuplot-contact* :command (format nil "replot \"~a\" u 1:~a with line lw 5 notitle" plot-file-name (+ j 2)))))
	  )))
    ;;(warn "press [Y] key for delete plotting window~%")
    ;;(while (not (y-or-n-p)))
    ;;(send *gnuplot-contact* :quit)
    ))

;; 2dcoord ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun plot-2dcoord (&key (data *2dcoord-list*) (wait nil) (with-title nil))

  (unless (boundp '*gnuplot-2dcoord*)
    (setq *gnuplot-2dcoord* (gnuplot)))

  (let* ((plot-file-name "log/tmp-log-for-plot-2dcoord.txt")
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
    (send *gnuplot-2dcoord* :command "set key outside")
    (send *gnuplot-2dcoord* :command "set grid")
    (send *gnuplot-2dcoord* :command "set xrange [-5:5]")
    (send *gnuplot-2dcoord* :command "set yrange [-5:5]")
    (cond (with-title
	   (send *gnuplot-2dcoord* :command (format nil "plot \"~a\" u ~a:~a with line lw 5 title \"1\"" plot-file-name 2 3))
	   (send *gnuplot-2dcoord* :command (format nil "replot \"~a\" u ~a:~a:~a:~a every ~a with vector title \"1\"" plot-file-name 2 3 4 5 plot-vect-interval)))
	  (t
	   (send *gnuplot-2dcoord* :command (format nil "plot \"~a\" u ~a:~a with line lw 5 notitle" plot-file-name 2 3))
	   (send *gnuplot-2dcoord* :command (format nil "replot \"~a\" u ~a:~a:~a:~a every ~a with vector notitle" plot-file-name 2 3 4 5 plot-vect-interval))))
    (when (> (length data) 1)
      (dotimes (j (length data))
    	(unless (equal j 0)
    	  (when wait
    	    (warn "press any key for plotting next line~%")
    	    (read-line))
	  (cond (with-title
		 (send *gnuplot-2dcoord* :command (format nil "replot \"~a\" u ~a:~a with line lw 5 title \"~a\""
							  plot-file-name (+ (* j 4) 2) (+ (* j 4) 3) (+ j 1)))
		 (send *gnuplot-2dcoord* :command (format nil "replot \"~a\" u ~a:~a:~a:~a every ~a with vector title \"~a\""
							  plot-file-name (+ (* j 4) 2) (+ (* j 4) 3) (+ (* j 4) 4) (+ (* j 4) 5) plot-vect-interval (+ j 1))))
		(t
		 (send *gnuplot-2dcoord* :command (format nil "replot \"~a\" u ~a:~a with line lw 5 notitle"
							  plot-file-name (+ (* j 4) 2) (+ (* j 4) 3)))
		 (send *gnuplot-2dcoord* :command (format nil "replot \"~a\" u ~a:~a:~a:~a every ~a with vector notitle"
							  plot-file-name (+ (* j 4) 2) (+ (* j 4) 3) (+ (* j 4) 4) (+ (* j 4) 5) plot-vect-interval))))

    	  )))
    ;;(warn "press [Y] key for delete plotting window~%")
    ;;(while (not (y-or-n-p)))
    ;;(send *gnuplot-2dcoord* :quit)
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

