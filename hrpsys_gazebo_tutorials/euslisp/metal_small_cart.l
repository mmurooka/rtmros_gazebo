(defun make-metal-small-cart
  ()
  (instance metal-small-cart :init))

(defclass metal-small-cart
  :super cascaded-link
  :slots ()
  )

(defmethod metal-small-cart
  (:init
   (&rest args)
   (send-super* :init :name "metal_small_cart" args)
   (setq handle nil)
   ;; 1. make links links and assoc all links
   (let* ((base (send self :make-base-link))
	  (base-pos (float-vector 0 0 0)))
     (send base :translate base-pos :world)
     ;; 2. assoc links
     ;;    Root link should be associated with "self".
     (send self :assoc base)
     ;; 3. make all joints
     ;;    Before making joints, you should :assoc all links.
     ;; 4. define slots for robot class
     ;;    links and joint-list for cascaded-link.
     (setq links (list base))
     (setq joint-list nil)
     ;; 5. call :init-ending after defining links and joint-list and return "self"
     (send self :init-ending)
     self)
   )
  ;; methods to define base-link ;;;;;;;;;;;;;;;;; ;;;;;;;;;;;;;;;;;
  (:make-base-link
   ()
   (let* (base-link base-link-list
		    left-handle right-handle
		    front-left-vertex front-right-vertex rear-left-vertex rear-right-vertex
		    cage
		    wh1 wh2 wh3 wh4 wh1in wh2in wh3in wh4in
		    bar-left bar-right bar-top
		    (box-l 375)
		    (box-w 345)
		    (box-h 385)
		    (box-thickness 20)
		    (wheel-z 40)
		    (wheel-r (/ 125 2))
		    )
     ;; cage
     (let* ((outer-box-lwh (list box-l box-w box-h))
	    (outer-box (apply #'make-cube outer-box-lwh))
	    (inner-box-lwh
	     (mapcar #'(lambda (x) (- x box-thickness)) (list 375 345 385)))
	    (inner-box (apply #'make-cube inner-box-lwh))
	    (pipe-num 10)
	    (pipe-thickness 10)
	    (pipe-interval (/ (- box-h (* pipe-num pipe-thickness)) pipe-num))
	    )
       ;; without pipe, just plate
       (send inner-box :locate (float-vector 0 0 (1+ box-thickness)) :world)
       (setq cage (body- outer-box inner-box))
       ;; with pipe
       (dotimes (i pipe-num)
	 (let* ((counter-pipe-lwh (list (- box-l (* box-thickness 2)) (+ box-w 20) pipe-interval))
		(counter-pipe (apply #'make-cube counter-pipe-lwh)))
	   (send counter-pipe :locate (float-vector 0 0 (- (* i (+ pipe-thickness pipe-interval)) (/ box-h 2))) :world)
	   (setq cage (body- cage counter-pipe))))
       (dotimes (i pipe-num)
	 (let* ((counter-pipe-lwh (list (+ box-l 20) (- (- box-w (* box-thickness 2)) 10) pipe-interval))
		(counter-pipe (apply #'make-cube counter-pipe-lwh)))
	   (send counter-pipe :locate (float-vector 0 0 (+ (- (* i (+ pipe-thickness pipe-interval)) (/ box-h 2)) 10)) :world)
	   (unless (equal i 0)
	     (setq cage (body- cage counter-pipe)))
	   ))
       (push-back cage base-link-list))
     ;; wheel
     (let* ((wh-h 30)
	    (wh-r-in (/ wheel-r 4))
	    (wh-h-in (+ wh-h 5))
	    (wh1-pos (float-vector (- (/ box-l 2) wheel-r) (+ (/ box-w 2)) (- (- (/ box-h 2)) wheel-z)))
	    (wh2-pos (float-vector (- (/ box-l 2) wheel-r) (- (/ box-w 2)) (- (- (/ box-h 2)) wheel-z)))
	    (wh3-pos (float-vector (+ (- (/ box-l 2)) (/ wheel-r 2)) (+ (/ box-w 2)) (- (- (/ box-h 2)) wheel-z)))
	    (wh4-pos (float-vector (+ (- (/ box-l 2)) (/ wheel-r 2)) (- (/ box-w 2)) (- (- (/ box-h 2)) wheel-z)))
	    )
       ;; wh1
       (setq wh1 (make-cylinder wheel-r wh-h))
       (send wh1 :orient -pi/2 :x)
       (send wh1 :locate wh1-pos :world)
       (setq wh1in (make-cylinder wh-r-in wh-h-in))
       (send wh1in :orient -pi/2 :x)
       (send wh1in :locate wh1-pos :world)
       ;; wh2
       (setq wh2 (make-cylinder wheel-r wh-h))
       (send wh2 :orient pi/2 :x)
       (send wh2 :locate wh2-pos :world)
       (setq wh2in (make-cylinder wh-r-in wh-h-in))
       (send wh2in :orient pi/2 :x)
       (send wh2in :locate wh2-pos :world)
       ;; wh3
       (setq wh3 (make-cylinder wheel-r wh-h))
       (send wh3 :orient -pi/2 :x)
       (send wh3 :locate wh3-pos :world)
       (setq wh3in (make-cylinder wh-r-in wh-h-in))
       (send wh3in :orient -pi/2 :x)
       (send wh3in :locate wh3-pos :world)
       ;; wh4
       (setq wh4 (make-cylinder wheel-r wh-h))
       (send wh4 :orient pi/2 :x)
       (send wh4 :locate wh4-pos :world)
       (setq wh4in (make-cylinder wh-r-in wh-h-in))
       (send wh4in :orient pi/2 :x)
       (send wh4in :locate wh4-pos :world)
       ;; add
       (push-back wh1 base-link-list)
       (push-back wh2 base-link-list)
       (push-back wh3 base-link-list)
       (push-back wh4 base-link-list)
       (push-back wh1in base-link-list)
       (push-back wh2in base-link-list)
       (push-back wh3in base-link-list)
       (push-back wh4in base-link-list)
       ;; set handle
       (setq front-left-vertex
	     (make-cascoords :coords
			     (make-coords :pos (float-vector (- (/ box-l 2) wheel-r) (+ (/ box-w 2)) (- (+ (/ box-h 2) wheel-z wheel-r))))
			     :name :front-left-vertex))
       (setq front-right-vertex
	     (make-cascoords :coords
			     (make-coords :pos (float-vector (- (/ box-l 2) wheel-r) (- (/ box-w 2)) (- (+ (/ box-h 2) wheel-z wheel-r))))
			     :name :front-right-vertex))
       (setq rear-left-vertex
	     (make-cascoords :coords
			     (make-coords :pos (float-vector (+ (- (/ box-l 2)) (/ wheel-r 2)) (+ (/ box-w 2)) (- (+ (/ box-h 2) wheel-z wheel-r))))
			     :name :rear-left-vertex))
       (setq rear-right-vertex
	     (make-cascoords :coords
			     (make-coords :pos (float-vector (+ (- (/ box-l 2)) (/ wheel-r 2)) (- (/ box-w 2)) (- (+ (/ box-h 2) wheel-z wheel-r))))
			     :name :rear-right-vertex))
       ;; add handle
       (send wh1 :assoc front-left-vertex)
       (push-back front-left-vertex handle)
       (send wh2 :assoc front-right-vertex)
       (push-back front-right-vertex handle)
       (send wh3 :assoc rear-left-vertex)
       (push-back rear-left-vertex handle)
       (send wh4 :assoc rear-right-vertex)
       (push-back rear-right-vertex handle)
       )
     ;; bar
     (let* ((bar-thikness 15)
	    (bar-length 800)
	    (bar-angle 5)
	    (bar-length-top 600)
	    (bar-thikness-top 12.5)
	    (bar-left-pos (float-vector (+ (- (/ box-l 2)) 25) (+ (/ box-w 2) (/ box-thickness 2)) (/ box-h 2)))
	    (bar-right-pos (float-vector (+ (- (/ box-l 2)) 25) (- (+ (/ box-w 2) (/ box-thickness 2))) (/ box-h 2)))
	    (bar-top-pos (float-vector (- (+ (- (/ box-l 2)) 25) (* (sin (deg2rad bar-angle)) (/ bar-length 2)))
				       (+ (/ bar-length-top 2)) (+ (* (cos (deg2rad bar-angle)) (/ bar-length 2)) (/ box-h 2))))
	    (handle-offset 60)
	    )
       ;; left
       (setq bar-left (make-cube bar-thikness bar-thikness bar-length))
       (send bar-left :orient (- (deg2rad bar-angle)) :y)
       (send bar-left :locate bar-left-pos :world)
       ;; right
       (setq bar-right (make-cube bar-thikness bar-thikness bar-length))
       (send bar-right :orient (- (deg2rad bar-angle)) :y)
       (send bar-right :locate bar-right-pos :world)
       ;; top
       (setq bar-top (make-cylinder bar-thikness-top bar-length-top))
       (send bar-top :orient pi/2 :x)
       (send bar-top :locate bar-top-pos :world)
       ;; add
       (push-back bar-left base-link-list)
       (push-back bar-right base-link-list)
       (push-back bar-top base-link-list)
       ;; set handle
       (setq left-handle
	     (make-cascoords :coords
			     (send (send (send bar-top :copy-worldcoords)
					 :translate (float-vector 0 0 (+ handle-offset))) :rotate 0 :x)
			     :name :left-handle))
       (setq right-handle
	     (make-cascoords :coords
			     (send (send (send bar-top :copy-worldcoords)
					 :translate (float-vector 0 0 (- bar-length-top handle-offset))) :rotate -pi :x)
			     :name :right-handle))
       ;; add handle
       (send bar-top :assoc left-handle)
       (push-back left-handle handle)
       (send bar-top :assoc right-handle)
       (push-back right-handle handle)
       )
     ;; set list and assoc
     (dolist (b (cdr base-link-list)) (send (car base-link-list) :assoc b))
     ;; translate
     (send (car base-link-list) :locate (float-vector 0 0 (+ (/ box-h 2) wheel-z wheel-r)))
     (send (car base-link-list) :orient pi :z)
     ;; make bodyset-link ;;;;;;;;;;;;;;;;;
     (setq base-link (instance bodyset-link :init (make-cascoords)
			       :bodies (append base-link-list nil)
			       :name :base-link))
     (dolist (l (send base-link :bodies))
       (send l :set-color :gray30))
     (dolist (l (list cage bar-left bar-right))
       (send l :set-color :gray80))
     (dolist (l (list wh1in wh2in wh3in wh4in))
       (send l :set-color :white))
     (dolist (l (list bar-top))
       (send l :set-color :brown))
     base-link))
  (:left-handle () (find-if #'(lambda (h) (eq (send h :name) :left-handle)) handle))
  (:right-handle () (find-if #'(lambda (h) (eq (send h :name) :right-handle)) handle))
  (:front-left-vertex () (find-if #'(lambda (h) (eq (send h :name) :front-left-vertex)) handle))
  (:front-right-vertex () (find-if #'(lambda (h) (eq (send h :name) :front-right-vertex)) handle))
  (:rear-left-vertex () (find-if #'(lambda (h) (eq (send h :name) :rear-left-vertex)) handle))
  (:rear-right-vertex () (find-if #'(lambda (h) (eq (send h :name) :rear-right-vertex)) handle))
  )
