#!/usr/bin/env roseus

#-:jsk
(jsk)
#-:rbrain-basic
(rbrain)


(load "package://hrpsys_gazebo_tutorials/euslisp/gazebo-plugin-utils.l")
(load "package://hrpsys_gazebo_tutorials/euslisp/gazebo-timer.l")


(defun usage-util
  ()

  (warn ";; util ;;;;;;;;;;;;;;;;;~%")
  (warn "(init-variable)~%")
  (warn "(save-variable)~%")
  (warn "(load-variable)~%")
  )


;; init ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun init-variable
  ()

  ;; result
  (setq *manip-list* nil)
  (setq *contact-list* nil)
  (setq *2dcoord-list* nil)
  (setq *eval-list* nil)
  (setq *fv-list* nil)
  (setq *class-list* nil)

  ;; final result
  (setq *all-orig-data* nil)
  (setq *all-class-data* nil)
  (setq *class-eval* nil)
  (setq *class-fv* nil)

  ;; manip meta parameter
  (init-manip-param)

  ;; strategy
  (setq *strategy-manip-list* nil)
  (gen-all-strategy)

  ;; init contact
  (init-contact)

  nil)

(defun init-variable-test
  ()

  ;; test result
  (setq *contact-list-test* nil)
  (setq *2dcoord-list-test* nil)
  (setq *eval-list-test* nil)
  (setq *fv-list-test* nil)

  ;; manip meta parameter
  (init-manip-param)

  ;; init contact
  (init-contact)

  nil)

(defun init-manip-param
  ()

  ;; manip meta parameter
  (setq *manip-cnt* 0)
  (setq *manip-dt* 0.01)
  (setq *manip-loop-num* 300)
  (setq *update-force-loop-num* 150)
  (setq *offset-z-list* nil)

  nil)

;; save/load ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun save-variable
  ()

  (dump-loadable-structure
   (format nil "log/log-~a.l"
	   (string-downcase (concatenate string (remove 58 (remove 32 (butlast (concatenate cons (unix:asctime (unix:localtime)))))))))
   *manip-list* *contact-list* *2dcoord-list*))

(defun load-variable
  ()

  (load (format nil "./~a" (string-downcase (string (read (piped-fork "ls -t1 log/*.l | head -n 1"))))))
  )

;; utill ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun get-class-num
  ()

  (length (elt *all-class-data* 1)))

;; update ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; obj coord
(defun update-coord-cupboard
  (&key (update-flag nil))

  (dotimes (i (length *objlist*))
    (when (or (not update-flag) (elt update-flag i))
      (update-coord-cupboard-one
       :obj (elt *objlist* i) :gp (elt *gplist* i)
       :offset-z (cond (*offset-z-list* (elt *offset-z-list* i))
		       (t 0)))))
  )

(defun update-coord-cupboard-one
  (&key (obj (car *objlist*)) (gp (car *gplist*)) (offset-z 0))

  ;; set gazebo model pose to eus model
  (let* ((gzb-pose (send (send gp :get-pose) :copy-worldcoords))
	 (trans-gzb-pose (send gzb-pose :translate (float-vector 0 0 offset-z) :local))
	 (gzb-pos (send trans-gzb-pose :worldpos)))
    (send obj :newcoords trans-gzb-pose)
    ;; (send obj :newcoords (make-coords :pos (float-vector 0 0 (elt gzb-pos 2)) :rot (send gzb-pose :rot))) ;; ignore x,y motion
    ))

;; ar coord
(defun update-ar-coord
  (f1 f2 &key (update-flag nil))

  (dotimes (i (length *objlist*))
    (when (or (not update-flag) (elt update-flag i))
      (eval (list 'update-ar-coord-one 'f1 'f2
		  :lhand (read-from-string (format nil "*lhand~d*" (+ i 1)))
		  :rhand (read-from-string (format nil "*rhand~d*" (+ i 1)))
		  :left-ar (read-from-string (format nil "*left-ar~d*" (+ i 1)))
		  :right-ar (read-from-string (format nil "*right-ar~d*" (+ i 1))))))))

(defun update-ar-coord-one
  (f1 f2 &key (lhand *lhand*) (rhand *rhand*) (left-ar *left-ar*) (right-ar *right-ar*))
  
  (let* ((l1 (* (+ (norm f1) 2) 5))
	 (f11n (normalize-vector f1))
	 (f12n (v* f11n (float-vector 1 1 1)))
	 (f13n (v* f11n f12n))
	 (f1coord (make-coords :pos (send (send lhand :copy-worldcoords) :worldpos) :rot (send (send (send lhand :copy-worldcoords) :transform (make-coords :rot (transpose (matrix f11n f12n f13n)))) :rot)))
	 (l2 (* (+ (norm f2) 2) 5))
	 (f21n (normalize-vector f2))
	 (f22n (v* f21n (float-vector 1 1 1)))
	 (f23n (v* f21n f22n))
	 (f2coord (make-coords :pos (send (send rhand :copy-worldcoords) :worldpos) :rot (send (send (send rhand :copy-worldcoords) :transform (make-coords :rot (transpose (matrix f21n f22n f23n)))) :rot))))
    (send left-ar :newcoords (send (send (send (send f1coord :copy-worldcoords) :rotate -pi/2 :z) :rotate 0 :x) :translate (float-vector 0 l1 0) :local))
    (send right-ar :newcoords (send (send (send (send f2coord :copy-worldcoords) :rotate -pi/2 :z) :rotate 0 :x) :translate (float-vector 0 l2 0) :local))
    ))

;; init contact
(defun init-contact
  ()

  (setq *offset-z-list* nil)
  ;; set force and pose
  (send-all *gplist* :add-force2 (float-vector 0 0 0) (float-vector 0 0 0))
  (send-all *gplist* :set-pose (make-coords))
  ;; wait
  (sleep-gazebo 1)
  ;; update coord
  (update-coord-cupboard)
  (send *pickview* :draw-objects)
  ;; calc contact thre
  (setq *offset-z-list*
	(mapcar #'- (update-contact-cupboard :return-contact-z t)))
  ;; update coord again
  (update-coord-cupboard)
  (send *pickview* :draw-objects)
  )

;; contact
(defun update-contact-cupboard
  (&key (update-flag nil) (return-contact-z nil))

  (let* (ret-tmp (ret nil))
    (dotimes (i (length *objlist*))
      (when (or (not update-flag) (elt update-flag i))
	(setq ret-tmp
	      (update-contact-cupboard-one
	       :obj (elt *objlist* i) :return-contact-z return-contact-z)))
      (when return-contact-z
	(push-back (/ (reduce #'+ ret-tmp) (length ret-tmp)) ret))
      )
    ret))

(defun update-contact-cupboard-one
  (&key (obj *obj1*) (return-contact-z nil))

  (let* ((bottom-face (send obj :faces))
	 (bottom-edges (send (elt bottom-face 1) :edges))
	 (bottom-vertices (mapcar #'(lambda (l) (car (send l :vertices))) bottom-edges))
	 (bottom-vertices-z (mapcar #'(lambda (v) (elt v 2)) bottom-vertices))
	 bottom-vcont bottom-lcont
	 (contact-z-thre 50)
	 ;;(contact-z-thre -0.5)
	 )
    (dotimes (i (length bottom-vertices-z))
      (push-back (< (elt bottom-vertices-z i) contact-z-thre) bottom-vcont)
      )
    (dotimes (i (length bottom-vertices-z))
      (push-back (and (elt bottom-vcont i) (elt bottom-vcont (mod (+ i 1) 4))) bottom-lcont)
      )
    ;;(print bottom-vertices-z)
    (send obj :put :vcont bottom-vcont)
    (send obj :put :lcont bottom-lcont)
    (send obj :put :fcont (not (position nil bottom-vcont)))
    (send obj :put :2d-pos (scale 0.001 (scale (/ 1.0 (length bottom-vertices)) (reduce #'v+ bottom-vertices))))
    (send obj :put :2d-angle (elt (car (send (send (send obj :copy-worldcoords) :rotate pi :z) :rpy-angle)) 0))
    (cond ((send obj :get :fcont) (send obj :put :contact :face))
	  ((position t (send obj :get :lcont)) (send obj :put :contact :line))
	  ((position t (send obj :get :vcont)) (send obj :put :contact :vertex))
	  (t (send obj :put :contact :nil)))
    (cond (return-contact-z bottom-vertices-z)
	  (t nil))
    ))

;; draw function ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; once
(defun draw-contact-cupboard
  (&key (update-flag nil))

  (dotimes (i (length *objlist*))
    (when (or (not update-flag) (elt update-flag i))
      (let* ((obj (elt *objlist* i))
	     (pos (float-vector 10 (+ 20 (* i 15))))
	     (output-string
	      (format nil "~a : ~a" i (string (send obj :get :contact)))))
	(draw-contact-cupboard-one :obj obj :pos pos :output-string output-string)))
    ))

(defun draw-contact-cupboard-one
  (&key (obj *obj1*) (pos (float-vector 10 20)) (output-string (string (send obj :get :contact))))

  (let* ((bottom-face (send obj :faces))
	 (bottom-edges (send (elt bottom-face 1) :edges))
	 (bottom-vertices (mapcar #'(lambda (l) (car (send l :vertices))) bottom-edges)))
    (dotimes (i (length bottom-vertices))
      (when (elt (send obj :get :vcont) i)
	(send (elt bottom-vertices i) :draw-on :color #f(1 0 0) :width 5))
      (when (elt (send obj :get :lcont) i)
	(send (elt bottom-edges i) :draw-on :color #f(1 0 0) :width 5))
      ))
  (send *pickview* :viewer :viewsurface
	:string (elt pos 0) (elt pos 1) output-string)
  )

;; loop
(defun update-draw-contact-loop ()
  (do-until-key
   (update-coord-cupboard)
   (update-contact-cupboard)
   (send *pickview* :draw-objects :flush nil)
   (draw-contact-cupboard)
   (send *viewer* :flush)
   (x::window-main-one)))

;; thread
(defun set-thread-update-draw-contact ()
  (sys:make-thread 1)
  (sys:thread #'update-draw-contact-loop))




;; gazebo plugin function ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defmethod gazebo-plugin-obj-operate
  ;; initialize
  (:init
   (&optional (_objname ""))
   (setq objname _objname)
   (ros::roseus "GazeboPluginUtilsEus")
   (ros::rate 100)		   
   (send self :init-add-force2)
   (send self :init-set-vel)
   (send self :init-get-vel)
   (send self :init-set-vel)
   (send self :update))
  ;; add-froce plugin ;;;;;;
  (:init-add-force2
   ()
   (ros::roseus "AddForceEus")
   (ros::advertise (format nil "~a~a" objname "/AddForcePlugin/ForceCommand1") geometry_msgs::wrench 1)
   (ros::advertise (format nil "~a~a" objname "/AddForcePlugin/ForceCommand2") geometry_msgs::wrench 1))
  (:add-force2-1
   (_f _t)
   (let* ((msg (instance geometry_msgs::wrench :init)))
     (send msg :force :x (elt _f 0))
     (send msg :force :y (elt _f 1))
     (send msg :force :z (elt _f 2))
     (send msg :torque :x (elt _t 0))
     (send msg :torque :y (elt _t 1))
     (send msg :torque :z (elt _t 2))
     (ros::publish (format nil "~a~a" objname "/AddForcePlugin/ForceCommand1") msg)))
  (:add-force2-2
   (_f _t)
   (let* ((msg (instance geometry_msgs::wrench :init)))
     (send msg :force :x (elt _f 0))
     (send msg :force :y (elt _f 1))
     (send msg :force :z (elt _f 2))
     (send msg :torque :x (elt _t 0))
     (send msg :torque :y (elt _t 1))
     (send msg :torque :z (elt _t 2))
     (ros::publish (format nil "~a~a" objname "/AddForcePlugin/ForceCommand2") msg)))
  (:add-force2
   (_f1 _f2 &optional (_t1 (float-vector 0 0 0)) (_t2 (float-vector 0 0 0)))
   (when _f1 (send self :add-force2-1 _f1 _t1))
   (when _f2 (send self :add-force2-2 _f2 _t2)))
  (:reset-force2
   ()
   (send self :add-force2 (float-vector 0 0 0) (float-vector 0 0 0)))
  )

;; hand controller function ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun set-hand-pose
  (&optional (pose :hook-pose) (arm :arms))

  (when (or (equal arm :larm) (equal arm :lhand) (equal arm :arms) (equal arm :hands))
    (send *lhand* pose))
  (when (or (equal arm :rarm) (equal arm :rhand) (equal arm :arms) (equal arm :hands))
    (send *rhand* pose))
  (cond 
   ((or (equal arm :larm) (equal arm :lhand))
    (send *hc* :send-command (list (send *lhand* :angle-vector) nil)))
   ((or (equal arm :rarm) (equal arm :rhand))
    (send *hc* :send-command (list nil (send *rhand* :angle-vector))))
   ((or (equal arm :arms) (equal arm :hands))
    (send *hc* :send-command (list (send *lhand* :angle-vector) (send *rhand* :angle-vector)))))
  )

;; model function ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun make-73b2-cupboard-without-all-door
  (&key (name "73b2-cupboard-without-door"))

  (let* ((d 353) (w 586) (h 1816)
	 (d/2 (/ d 2)) (w/2 (/ w 2)) (-w/2 (/ w -2))
	 (handle-offset 850)
	 c)
    (setq c (instance askul-cupboard :init d w h
		      :name name
		      :panel-heights (list 760 880
					   (+ 870 (* 230 1))
					   (+ 870 (* 230 2)))))
    (send c :rotate pi :z :local)
    (send c :add-handle (make-cascoords :pos (float-vector (+ (/ d 2) 180) (- (- (/ w 2)) 30) (- h handle-offset))
					:rot (rpy-matrix pi -pi/2 0) :name :left-front-handle1))
    (send c :add-handle (make-cascoords :pos (float-vector (+ (/ d 2) 180) (+ (+ (/ w 2)) 30) (- h handle-offset))
					:rot (rpy-matrix pi -pi/2 0) :name :right-front-handle1))
    (send c :add-handle (make-cascoords :pos (float-vector (/ d 2) (/ w 2) 0)
					:rot (unit-matrix 3) :name :left-front-edge))
    (send c :add-handle (make-cascoords :pos (float-vector (/ d 2) (- (/ w 2)) 0)
					:rot (unit-matrix 3) :name :right-front-edge))
    (send c :add-handle (make-cascoords :pos (float-vector (- (/ d 2)) (/ w 2) 0)
					:rot (unit-matrix 3) :name :left-rear-edge))
    (send c :add-handle (make-cascoords :pos (float-vector (- (/ d 2)) (- (/ w 2)) 0)
					:rot (unit-matrix 3) :name :right-rear-edge))
    c))

;; timer ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun sleep-gazebo
  (wait-time) ;; [sec]
  (send *gazebo-timer* :wait wait-time)
  )

(defun sleep-gazebo-init
  (wait-time) ;; [sec]
  (send *gazebo-timer* :wait-without-block-init wait-time)
  )

(defun sleep-gazebo-wait
  ()
  (send *gazebo-timer* :wait-without-block-wait)
  )


;; mass ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun sqrt-with-sgn
  (x)

  (cond ((eps= (float x) 0.0) 0.0)
	(t (* (/ x (abs x)) (sqrt (abs x)))))
  )
