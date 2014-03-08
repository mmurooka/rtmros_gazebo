#!/usr/bin/env roseus

(ros::load-ros-manifest "rosgraph_msgs")


(defun init-gazebo-timer
  ()
  (instance gazebo-timer :init))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; class definition

(defclass gazebo-timer
  :super propertied-object
  :slots (init-time latest-time wait-finish-time wait-finish-time-without-block) ;; [sec]
  )

(defmethod gazebo-timer
  ;; initialize
  (:init
   ()
   (ros::roseus "GazeboTimer")
   (ros::rate 1000)
   (ros::subscribe "/clock" rosgraph_msgs::Clock #'send self :time-cb)
   (send self :update))
  (:set-init-time
   ()
   (send self :update)
   (setq init-time latest-time))
  ;; update
  (:update
   ()
   (ros::spin-once))
  (:time-cb
   (msg)
   (setq latest-time (send (send msg :clock) :to-sec)))
  ;; get
  (:get-raw-time
   ()
   latest-time)
  (:get-time
   ()
   (- latest-time init-time))
  ;; wait
  (:wait
   (wait-time) ;; [sec]
   (send self :update)
   (setq wait-finish-time (+ latest-time wait-time))
   (while t
     (send self :update)
     (when (> latest-time wait-finish-time)
       (return-from nil nil))))
  (:wait-without-block-init
   (wait-time) ;; [sec]
   (send self :update)
   (setq wait-finish-time-without-block (+ latest-time wait-time)))
  (:wait-without-block-wait
   ()
   (while t
     (send self :update)
     (when (> latest-time wait-finish-time-without-block)
       (return-from nil nil))))
  )
