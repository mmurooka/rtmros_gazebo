#!/usr/bin/env roseus

#-:jsk
(jsk)
#-:rbrain-basic
(rbrain)


(load "package://hrpsys_gazebo_tutorials/euslisp/manip-obj-with-hand-util.l")


(defun update-draw-contact-loop ()
  (do-until-key
   (update-coord-cupboard)
   (update-contact-cupboard)
   (send *pickview* :draw-objects :flush nil)
   (draw-contact-cupboard)
   (send *viewer* :flush)
   (x::window-main-one)))

(defun set-thread-update-draw-contact ()
  (sys:make-thread 1)
  (sys:thread #'update-draw-contact-loop))

;; pull slide ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun pull-slide ()
  (send-all *gplist* :set-pose (make-coords :rpy (list pi 0 0)))
  (send-all *gplist* :add-force2 (float-vector 10 0 2) (float-vector 10 0 2))
  (update-draw-contact-loop)
  (send-all *gplist* :add-force2 (float-vector 0 0 0) (float-vector 0 0 0))
  )

;; push slide ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun push-slide ()
  (send-all *gplist* :set-pose (make-coords :rpy (list pi 0 0)))
  (send-all *gplist* :add-force2 (float-vector -5 0 -5) (float-vector -5 0 -5))
  (update-draw-contact-loop)
  (send-all *gplist* :add-force2 (float-vector 0 0 0) (float-vector 0 0 0))
  )

;; pull tile ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun pull-tilt ()
  (send-all *gplist* :set-pose (make-coords :rpy (list pi 0 0)))
  (dotimes (i 4)
    (pull-tilt-right)
    (pull-tilt-left))
  )

(defun pull-tilt-right ()
  ;;(send-all *gplist* :add-force2 (float-vector -10 0 -2) (float-vector 22 0 2))
  (send-all *gplist* :add-force2 (float-vector -10 0 -2) (float-vector 20 1 2))
  (update-draw-contact-loop)
  (send-all *gplist* :add-force2 (float-vector 0 0 0) (float-vector 0 0 0))
  )

(defun pull-tilt-left ()
  ;;(send-all *gplist* :add-force2 (float-vector 22 0 2) (float-vector -10 0 -2))
  (send-all *gplist* :add-force2 (float-vector 20 -1 2) (float-vector -10 0 -2))
  (update-draw-contact-loop)
  (send-all *gplist* :add-force2 (float-vector 0 0 0) (float-vector 0 0 0))
  )

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

;; lift up ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun lift-up ()
  (send-all *gplist* :set-pose (make-coords :rpy (list pi 0 0)))
  (send-all *gplist* :add-force2 (float-vector 25 0 0) (float-vector 25 0 0) (float-vector 0 -6.7 0) (float-vector 0 -6.7 0))
  ;;(send-all *gplist* :add-force2 (float-vector 20 0 0) (float-vector 20 0 0) (float-vector 0 -6 0) (float-vector 0 -6 0))
  (update-draw-contact-loop)
  (send-all *gplist* :add-force2 (float-vector 0 0 0) (float-vector 0 0 0))
  )
