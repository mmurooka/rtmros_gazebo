(ros::load-ros-manifest "hrpsys_gazebo_atlas")
(ros::roseus "get-pointcloud")

;;(setq topic-name "/openni/depth_registered/points_throttle")
;;(setq topic-name "/bw_pcl_nodelet/voxelgrid/output")
(setq divide-num 10)
(setq topic-name-list nil)
(dotimes (i divide-num)
  (push (format nil "/pcl_divider_nodelet/voxelgrid~A/output_update" (+ i 1))
	topic-name-list ))
	;;(setq topic-name "/pcl_divider_nodelet/voxelgrid/output_update")

(defun get-callback0 (msg)
  (let ((i 0))
    (format t "get-pointcloud~A~%" i)
    (setq hoge msg)
    (setf (elt *eus-pcs* i) (make-eus-pointcloud-from-ros-msg msg :remove-nan t))
    (unless (elt *eus-pcs* i)
      (setf (elt *eus-pcs* i) (instance pointcloud :init)))
    (format t "~A points" (car (array-dimensions (send (elt *eus-pcs* i) :points))))
    (send (elt *eus-pcs* i) :point-size 10)
    (set-color i)
    (send *irtviewer* :objects *eus-pcs*)
    ))

(defun get-callback1 (msg)
  (let ((i 1))
    (format t "get-pointcloud~A~%" i)
    (setq hoge msg)
    (setf (elt *eus-pcs* i) (make-eus-pointcloud-from-ros-msg msg :remove-nan t))
    (unless (elt *eus-pcs* i)
      (setf (elt *eus-pcs* i) (instance pointcloud :init)))
    (format t "~A points" (car (array-dimensions (send (elt *eus-pcs* i) :points))))
    (send (elt *eus-pcs* i) :point-size 10)
    (set-color i)
    (send *irtviewer* :objects *eus-pcs*)
    ))
(defun get-callback2 (msg)
  (let ((i 2))
    (format t "get-pointcloud~A~%" i)
    (setq hoge msg)
    (setf (elt *eus-pcs* i) (make-eus-pointcloud-from-ros-msg msg :remove-nan t))
    (unless (elt *eus-pcs* i)
      (setf (elt *eus-pcs* i) (instance pointcloud :init)))
    (format t "~A points" (car (array-dimensions (send (elt *eus-pcs* i) :points))))
    (send (elt *eus-pcs* i) :point-size 10)
    (set-color i)
    (send *irtviewer* :objects *eus-pcs*)
    ))
(defun get-callback3 (msg)
  (let ((i 3))
    (format t "get-pointcloud~A~%" i)
    (setq hoge msg)
    (setf (elt *eus-pcs* i) (make-eus-pointcloud-from-ros-msg msg :remove-nan t))
    (unless (elt *eus-pcs* i)
      (setf (elt *eus-pcs* i) (instance pointcloud :init)))
    (format t "~A points" (car (array-dimensions (send (elt *eus-pcs* i) :points))))
    (send (elt *eus-pcs* i) :point-size 10)
    (set-color i)
    (send *irtviewer* :objects *eus-pcs*)
    ))
(defun get-callback4 (msg)
  (let ((i 4))
    (format t "get-pointcloud~A~%" i)
    (setq hoge msg)
    (setf (elt *eus-pcs* i) (make-eus-pointcloud-from-ros-msg msg :remove-nan t))
    (unless (elt *eus-pcs* i)
      (setf (elt *eus-pcs* i) (instance pointcloud :init)))
    (format t "~A points" (car (array-dimensions (send (elt *eus-pcs* i) :points))))
    (send (elt *eus-pcs* i) :point-size 10)
    (set-color i)
    (send *irtviewer* :objects *eus-pcs*)
    ))
(defun get-callback5 (msg)
  (let ((i 5))
    (format t "get-pointcloud~A~%" i)
    (setq hoge msg)
    (setf (elt *eus-pcs* i) (make-eus-pointcloud-from-ros-msg msg :remove-nan t))
    (unless (elt *eus-pcs* i)
      (setf (elt *eus-pcs* i) (instance pointcloud :init)))
    (format t "~A points" (car (array-dimensions (send (elt *eus-pcs* i) :points))))
    (send (elt *eus-pcs* i) :point-size 10)
    (set-color i)
    (send *irtviewer* :objects *eus-pcs*)
    ))
(defun get-callback6 (msg)
  (let ((i 6))
    (format t "get-pointcloud~A~%" i)
    (setq hoge msg)
    (setf (elt *eus-pcs* i) (make-eus-pointcloud-from-ros-msg msg :remove-nan t))
    (unless (elt *eus-pcs* i)
      (setf (elt *eus-pcs* i) (instance pointcloud :init)))
    (format t "~A points" (car (array-dimensions (send (elt *eus-pcs* i) :points))))
    (send (elt *eus-pcs* i) :point-size 10)
    (set-color i)
    (send *irtviewer* :objects *eus-pcs*)
    ))
(defun get-callback7 (msg)
  (let ((i 7))
    (format t "get-pointcloud~A~%" i)
    (setq hoge msg)
    (setf (elt *eus-pcs* i) (make-eus-pointcloud-from-ros-msg msg :remove-nan t))
    (unless (elt *eus-pcs* i)
      (setf (elt *eus-pcs* i) (instance pointcloud :init)))
    (format t "~A points" (car (array-dimensions (send (elt *eus-pcs* i) :points))))
    (send (elt *eus-pcs* i) :point-size 10)
    (set-color i)
    (send *irtviewer* :objects *eus-pcs*)
    ))
(defun get-callback8 (msg)
  (let ((i 8))
    (format t "get-pointcloud~A~%" i)
    (setq hoge msg)
    (setf (elt *eus-pcs* i) (make-eus-pointcloud-from-ros-msg msg :remove-nan t))
    (unless (elt *eus-pcs* i)
      (setf (elt *eus-pcs* i) (instance pointcloud :init)))
    (format t "~A points" (car (array-dimensions (send (elt *eus-pcs* i) :points))))
    (send (elt *eus-pcs* i) :point-size 10)
    (set-color i)
    (send *irtviewer* :objects *eus-pcs*)
    ))
(defun get-callback9 (msg)
  (let ((i 9))
    (format t "get-pointcloud~A~%" i)
    (setq hoge msg)
    (setf (elt *eus-pcs* i) (make-eus-pointcloud-from-ros-msg msg :remove-nan t))
    (unless (elt *eus-pcs* i)
      (setf (elt *eus-pcs* i) (instance pointcloud :init)))
    (format t "~A points" (car (array-dimensions (send (elt *eus-pcs* i) :points))))
    (send (elt *eus-pcs* i) :point-size 10)
    (set-color i)
    (send *irtviewer* :objects *eus-pcs*)
    ))

(defun set-color (i)
  (cond ((= (mod i 3) 0)
	 (send (elt *eus-pcs* i) :point-color (float-vector 0 1 0)))
	((= (mod i 3) 1)
	 (send (elt *eus-pcs* i) :point-color (float-vector 1 1 1)))
	((= (mod i 3) 2)
	 (send (elt *eus-pcs* i) :point-color (float-vector 1 0 0))))
  )

(defun main
  ()
  (setq *eus-pcs* (make-list (length topic-name-list)))
  (dotimes (i (length topic-name-list))
    (setf (elt *eus-pcs* i) (instance pointcloud :init))
    )
  (objects *eus-pcs*)
  (ros::subscribe "/pcl_divider_nodelet/voxelgrid1/output_update"
		  sensor_msgs::PointCloud2 
		  #'get-callback0)
(ros::subscribe "/pcl_divider_nodelet/voxelgrid2/output_update"
		  sensor_msgs::PointCloud2 
		  #'get-callback1)
(ros::subscribe "/pcl_divider_nodelet/voxelgrid3/output_update"
		  sensor_msgs::PointCloud2 
		  #'get-callback2)
(ros::subscribe "/pcl_divider_nodelet/voxelgrid4/output_update"
		  sensor_msgs::PointCloud2 
		  #'get-callback3)
(ros::subscribe "/pcl_divider_nodelet/voxelgrid5/output_update"
		  sensor_msgs::PointCloud2 
		  #'get-callback4)
(ros::subscribe "/pcl_divider_nodelet/voxelgrid6/output_update"
		  sensor_msgs::PointCloud2 
		  #'get-callback5)
(ros::subscribe "/pcl_divider_nodelet/voxelgrid7/output_update"
		  sensor_msgs::PointCloud2 
		  #'get-callback6)
(ros::subscribe "/pcl_divider_nodelet/voxelgrid8/output_update"
		  sensor_msgs::PointCloud2 
		  #'get-callback7)
(ros::subscribe "/pcl_divider_nodelet/voxelgrid9/output_update"
		  sensor_msgs::PointCloud2 
		  #'get-callback8)
(ros::subscribe "/pcl_divider_nodelet/voxelgrid10/output_update"
		  sensor_msgs::PointCloud2 
		  #'get-callback9)
  (ros::rate 1)
  (do-until-key
   (print "loop")
   (ros::spin-once)
   (x::window-main-one)
   (ros::sleep)
   (unless (ros::ok) (return)))
  )

(main)

