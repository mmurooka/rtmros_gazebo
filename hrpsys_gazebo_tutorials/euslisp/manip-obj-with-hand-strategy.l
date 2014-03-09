#!/usr/bin/env roseus

#-:jsk
(jsk)
#-:rbrain-basic
(rbrain)


(load "package://hrpsys_gazebo_tutorials/euslisp/manip-obj-with-hand-util.l")


(defun usage-strategy
  ()

  (warn ";; strategy ;;;;;;;;;;;;;;;;;~%")
  (warn "(pull-slide)~%")
  (warn "(push-slide)~%")
  (warn "(pull-tilt)~%")
  (warn "(push-tilt)~%")
  (warn "(lift-up)~%")
  )


;; pull slide ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun pull-slide ()
  (send-all *gplist* :set-pose (make-coords :rpy (list pi 0 0)))
  (send-all *gplist* :add-force2 (float-vector 5 0 3) (float-vector 5 0 3))
  ;;(send-all *gplist* :add-force2 (float-vector 10 0 2) (float-vector 10 0 2))
  (update-draw-contact-loop)
  (send-all *gplist* :add-force2 (float-vector 0 0 0) (float-vector 0 0 0))
  )

(defun gen-pull-slide ()
  (setq *manip-pull-slide* nil)
  (dotimes (i *manip-loop-num*)
    (push-back (list (float-vector 10 0 2) (float-vector 10 0 2)) *manip-pull-slide*))
  (push-back *manip-pull-slide* *strategy-manip-list*))

;; push slide ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun push-slide ()
  (send-all *gplist* :set-pose (make-coords :rpy (list pi 0 0)))
  (send-all *gplist* :add-force2 (float-vector -1 0 -3) (float-vector -1 0 -3))
  ;;(send-all *gplist* :add-force2 (float-vector -5 0 -5) (float-vector -5 0 -5))
  (update-draw-contact-loop)
  (send-all *gplist* :add-force2 (float-vector 0 0 0) (float-vector 0 0 0))
  )

(defun gen-push-slide ()
  (setq *manip-push-slide* nil)
  (dotimes (i *manip-loop-num*)
    (push-back (list (float-vector -5 0 -5) (float-vector -5 0 -5)) *manip-push-slide*))
  (push-back *manip-push-slide* *strategy-manip-list*))

;; pull tile ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun pull-tilt ()
  (send-all *gplist* :set-pose (make-coords :rpy (list pi 0 0)))
  (dotimes (i 4)
    (pull-tilt-right)
    (pull-tilt-left))
  )

(defun pull-tilt-right ()
  ;;(send-all *gplist* :add-force2 (float-vector -10 0 -2) (float-vector 20 1 2))
  (send-all *gplist* :add-force2 (float-vector -10 0 -2) (float-vector 20 0 2))
  (update-draw-contact-loop)
  (send-all *gplist* :add-force2 (float-vector 0 0 0) (float-vector 0 0 0))
  )

(defun pull-tilt-left ()
  ;;(send-all *gplist* :add-force2 (float-vector 20 -1 2) (float-vector -10 0 -2))
  (send-all *gplist* :add-force2 (float-vector 20 0 2) (float-vector -10 0 -2))
  (update-draw-contact-loop)
  (send-all *gplist* :add-force2 (float-vector 0 0 0) (float-vector 0 0 0))
  )

(defun gen-pull-tilt ()
  (setq *manip-pull-tilt* nil)
  (dotimes (j 1)
    (dotimes (i (round (* *manip-loop-num* 0.4)))
      (push-back (list (float-vector -10 0 -2) (float-vector 20 1 2)) *manip-pull-tilt*))
    (dotimes (i (- *manip-loop-num* (round (* *manip-loop-num* 0.4))))
      (push-back (list (float-vector 20 -1 2) (float-vector -10 0 -2)) *manip-pull-tilt*))
    )
  (push-back *manip-pull-tilt* *strategy-manip-list*))

;; push tilt ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun push-tilt ()
  (send-all *gplist* :set-pose (make-coords :rpy (list pi 0 0)))
  (dotimes (i 4)
    (push-tilt-right)
    (push-tilt-left))
  )

(defun push-tilt-right ()
  (send-all *gplist* :add-force2 (float-vector -10 0 2) (float-vector 20 0 -2))
  (update-draw-contact-loop)
  (send-all *gplist* :add-force2 (float-vector 0 0 0) (float-vector 0 0 0))
  )

(defun push-tilt-left ()
  (send-all *gplist* :add-force2 (float-vector 20 0 -2) (float-vector -10 0 2))
  (update-draw-contact-loop)
  (send-all *gplist* :add-force2 (float-vector 0 0 0) (float-vector 0 0 0))
  )

(defun gen-push-tilt ()
  (setq *manip-push-tilt* nil)
  (dotimes (j 1)
    (dotimes (i (round (* *manip-loop-num* 0.4)))
      (push-back (list (float-vector -10 0 2) (float-vector 20 0 -2)) *manip-push-tilt*))
    ;;(dotimes (i (/ *manip-loop-num* 2))
    (dotimes (i (- *manip-loop-num* (round (* *manip-loop-num* 0.4))))
      (push-back (list (float-vector 20 0 -2) (float-vector -10 0 2)) *manip-push-tilt*))
    )
  (push-back *manip-push-tilt* *strategy-manip-list*))

;; lift up ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun lift-up ()
  (send-all *gplist* :set-pose (make-coords :rpy (list pi 0 0)))
  (send-all *gplist* :add-force2 (float-vector 25 0 0) (float-vector 25 0 0) (float-vector 0 -6.7 0) (float-vector 0 -6.7 0))
  ;;(send-all *gplist* :add-force2 (float-vector 20 0 0) (float-vector 20 0 0) (float-vector 0 -6 0) (float-vector 0 -6 0))
  (update-draw-contact-loop)
  (send-all *gplist* :add-force2 (float-vector 0 0 0) (float-vector 0 0 0))
  )

(defun gen-lift-up ()
  (setq *manip-lift-up* nil)
  (dotimes (i *manip-loop-num*)
    (push-back (list (float-vector 25 0 0) (float-vector 25 0 0)) *manip-lift-up*))
    ;;(push-back (list (float-vector 25 0 0) (float-vector 25 0 0) (float-vector 0 -6.7 0) (float-vector 0 -6.7 0)) *manip-lift-up*))
  (push-back *manip-lift-up* *strategy-manip-list*)
  )

;; all ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun gen-all-strategy ()
  (gen-pull-slide)
  (gen-push-slide)
  (gen-pull-tilt)
  (gen-push-tilt)
  (gen-lift-up))

