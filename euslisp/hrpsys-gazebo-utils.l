#-:jsk
(jsk)
#-:rbrain-basic
(rbrain)

(defun eus2urdf-for-gazebo (obj &optional name)
  (let* ((model-name (string-downcase (or name (send obj :name))))
	 (tmp-dir "/tmp"))
    ;; check model is named
    (unless (send obj :name)
      (warn "(send obj :name) is nil. set name to model.~%")
      (return-from eus2urdf-for-gazebo))
    ;; convert eus model to collada
    (convert-rbrain-model-to-collada obj :model-name model-name :output-directory tmp-dir)
    ;; call python script
    (unix:system (format nil "rosrun hrpsys_gazebo_tutorials eus2urdf_for_gazebo_pyscript.py ~a ~a/~a-robot.dae" model-name tmp-dir model-name))
    ))
(warn "~%(eus2urdf-for-gazebo (make-askul-gdh-cupboard) \"cupboard\")~%")


(defun eus2urdf-room-for-gazebo (room &key make-static make-world-file make-launch-file (robot-name "HRP2JSK"))
  (let* ((room-name (string-downcase (send room :name)))
	 (package-dir-path (string-downcase (read (piped-fork "rospack find hrpsys_gazebo_tutorials"))))
	 (world-src-file-path (format nil "~a/worlds/world_source.world" package-dir-path))
	 (world-new-file-path (format nil "~a/worlds/~a_~a.world" package-dir-path robot-name room-name))
	 (launch-src-file-path (format nil "~a/launch/gazebo_launch_source.launch" package-dir-path))
	 (launch-new-file-path (format nil "~a/launch/gazebo_~a_no_controllers_in_~a.launch" package-dir-path (string-downcase robot-name) room-name)))
    (when make-world-file
      ;; generate the world file
      (unix:system (format nil "rm -rf ~a" world-new-file-path))
      (unix:system (format nil "cp ~a ~a" world-src-file-path world-new-file-path))
      (unix:system (format nil "sed -i -e \"s@    <!-- A robot -->@    <!-- A robot -->\\n    <include>\\n      <uri>model://~a</uri>\\n      <pose>1.0 0 0.70 0 0 0</pose>\\n    </include>@g\" ~a" robot-name world-new-file-path)))
    (dolist (obj (send room :objects))
      (let* ((obj-name)
	     (obj-cnt 0))
	;; deal with the objects with no name. (objects with no name cause an error in converting)
	(cond ((null (send obj :name)) (send obj :name "no-name"))
	      ((symbolp (send obj :name)) (send obj :name (string-downcase (send obj :name)))))
	;; set obj-name
	(while t
	  (setq obj-name (string-downcase (format nil "~a-~a-~a" room-name (send obj :name) obj-cnt)))
	  (let* ((model-file-path (format nil "~a/environment_models/~a" package-dir-path obj-name)))
	    (unless (probe-file model-file-path) (return)))
	  (incf obj-cnt))
	;; convert the model
	(eus2urdf-for-gazebo obj obj-name)
	;; make static the model
	(when (or make-static make-world-file)
	  (unix:system (format nil "rosrun hrpsys_gazebo_tutorials make_static_model.py ~a" obj-name)))
	(when make-world-file
	  ;; add the obects to the world file
	  (let* ((pos (scale 0.001 (send obj :worldpos)))
		 (pos-string (format nil "~a ~a ~a" (elt pos 0) (elt pos 1) (elt pos 2)))
		 (rpy (car (rpy-angle (send obj :worldrot))))
		 (rpy-string (format nil "~a ~a ~a" (elt rpy 2) (elt rpy 1) (elt rpy 0))))
	    (unix:system (format nil "sed -i -e \"s@    <!-- Objects -->@    <!-- Objects -->\\n    <include>\\n      <uri>model://~a_static</uri>\\n      <pose>~a ~a</pose>\\n    </include>@g\" ~a" obj-name pos-string rpy-string world-new-file-path))))
	))
    ;; generate the model database file
    (unix:system (format nil "rosrun hrpsys_gazebo_tutorials generate_model_database.py"))
    (when make-launch-file
      ;; generate the launch file
      (unix:system (format nil "rm -rf ~a" launch-new-file-path))
      (unix:system (format nil "cp ~a ~a" launch-src-file-path launch-new-file-path))
      (unix:system (format nil "sed -i -e \"s@__world_name__@~a_~a.world@g\" ~a" robot-name room-name launch-new-file-path))
      (unix:system (format nil "sed -i -e \"s@__robot_name__@~a@g\" ~a" (string-downcase robot-name) launch-new-file-path)))
    (warning-message 2 "You can start simulation with following command:~%roslaunch hrpsys_gazebo_tutorials gazebo_~a_no_controllers_in_~a.launch~%" (string-downcase robot-name) room-name)
    ))
(warn "(eus2urdf-room-for-gazebo (room73b2))~%")


(defun generate-room-models (name)
  ;; check the function making room exits.
  (unless (functionp (read-from-string name))
    (warn "[ERROR] room (~a) is not found." name)
    (return-from generate-room-world))
  ;; check the model is derived from the room class
  (let* ((room (funcall (read-from-string name))))
    (unless (derivedp room scene)
      (warn "[ERROR] room (~a) is not room the model." name)
      (return-from generate-room-world))
    ;; convert room models
    (eus2urdf-room-for-gazebo room :make-static t :make-world-file t :make-launch-file t)
    ))
(warn "(generate-room-models \"room73b2\")~%")
