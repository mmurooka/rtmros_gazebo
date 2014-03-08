#!/usr/bin/env roseus

(ros::load-ros-manifest "rosgraph_msgs")


(defun init-gazebo-timer
  ()
  (instance gazebo-timer :init))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; class definition

(defclass gazebo-timer
  :super propertied-object
  :slots (init-time latest-time init-wait-time) ;; [sec]
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
   (setq init-wait-time latest-time)
   (while t
     (send self :update)
     (when (> latest-time (+ init-wait-time wait-time))
       (return-from nil nil))))
  )
