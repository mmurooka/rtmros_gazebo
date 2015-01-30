#!/usr/bin/env roseus

#-:jsk
(jsk)
#-:rbrain-basic
(rbrain)


(load "package://hrpsys_gazebo_tutorials/euslisp/manip-obj-with-hand-util.l")


(defun usage-generate
  ()

  (warn ";; generate ;;;;;;;;;;;;;;;;;~%")
  (warn "(generate) ;; generate default model~%")
  )


;; generate function ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; default
(defun generate
  ()

  (generate-cupboard))

;; box
(defun generate-box
  (&key (error-ratio nil))
  (let* ((obj-pp-list ;; pysical parameter list
	  (list
	   ;; default
	   (list (cons :mass (get-value-with-error 5 error-ratio))
		 (cons :origin (float-vector 0 0 0.5))
		 (cons :inertia (float-vector 10 5.7411 0.788))
		 (cons :mu1 (get-value-with-error 0.1 error-ratio))
		 (cons :mu2 (get-value-with-error 0.1 error-ratio))
		 (cons :fdir1 (float-vector 1 0 0))
		 (cons :kp 1000000.0)
		 (cons :kd 100.0))
	   ;; high friction
	   (list (cons :mass (get-value-with-error 5 error-ratio))
		 (cons :origin (float-vector 0 0 0.5))
		 (cons :inertia (float-vector 10 5.7411 0.788))
		 (cons :mu1 (get-value-with-error 50.0 error-ratio))
		 (cons :mu2 (get-value-with-error 50.0 error-ratio))
		 (cons :fdir1 (float-vector 1 0 0))
		 (cons :kp 1000000.0)
		 (cons :kd 100.0))
	   ;; heavy
	   (list (cons :mass (get-value-with-error 7.5 error-ratio))
		 (cons :origin (float-vector 0 0 0.5))
		 (cons :inertia (float-vector 10 5.7411 0.788))
		 (cons :mu1 (get-value-with-error 0.1 error-ratio))
		 (cons :mu2 (get-value-with-error 0.1 error-ratio))
		 (cons :fdir1 (float-vector 1 0 0))
		 (cons :kp 1000000.0)
		 (cons :kd 100.0))
	   ;; heavy, high friction
	   (list (cons :mass (get-value-with-error 7.5 error-ratio))
		 (cons :origin (float-vector 0 0 0.5))
		 (cons :inertia (float-vector 10 5.7411 0.788))
		 (cons :mu1 (get-value-with-error 50.0 error-ratio))
		 (cons :mu2 (get-value-with-error 50.0 error-ratio))
		 (cons :fdir1 (float-vector 1 0 0))
		 (cons :kp 1000000.0)
		 (cons :kd 100.0))
	   ;; light
	   (list (cons :mass (get-value-with-error 2.5 error-ratio))
		 (cons :origin (float-vector 0 0 0.5))
		 (cons :inertia (float-vector 10 5.7411 0.788))
		 (cons :mu1 (get-value-with-error 0.1 error-ratio))
		 (cons :mu2 (get-value-with-error 0.1 error-ratio))
		 (cons :fdir1 (float-vector 1 0 0))
		 (cons :kp 1000000.0)
		 (cons :kd 100.0))
	   ;; very heavy
	   (list (cons :mass (get-value-with-error 10 error-ratio))
		 (cons :origin (float-vector 0 0 0.5))
		 (cons :inertia (float-vector 10 5.7411 0.788))
		 (cons :mu1 (get-value-with-error 0.1 error-ratio))
		 (cons :mu2 (get-value-with-error 0.1 error-ratio))
		 (cons :fdir1 (float-vector 1 0 0))
		 (cons :kp 1000000.0)
		 (cons :kd 100.0))
	   ))
	 (cnt 0))
    (dolist (obj-pp obj-pp-list)
      (incf cnt)
      ;; set physical parameter to original urdf
      (unix::system 
       (format nil "sed -e 's@  </link>@    <inertial>\\n      <mass value=\"~a\" />\\n      <origin xyz=\"~a ~a ~a\" rpy=\"0 0 0\"/>\\n      <inertia ixx=\"~a\" ixy=\"0\" ixz=\"0\" iyy=\"~a\" iyz=\"0\" izz=\"~a\"/>\\n    </inertial>\\n  </link>\\n  <gazebo reference=\"nil_link\">\\n    <kp>~a</kp>\\n    <kd>~a</kd>\\n    <mu1>~a</mu1>\\n    <mu2>~a</mu2>\\n    <fdir1>~a ~a ~a</fdir1>\\n  </gazebo>@g' `rospack find hrpsys_gazebo_tutorials`/environment_models/box/model.urdf.in > `rospack find hrpsys_gazebo_tutorials`/environment_models/box/model~d.urdf"
	       (cdr (assoc :mass obj-pp)) (elt (cdr (assoc :origin obj-pp)) 0) (elt (cdr (assoc :origin obj-pp)) 1) (elt (cdr (assoc :origin obj-pp)) 2) (elt (cdr (assoc :inertia obj-pp)) 0) (elt (cdr (assoc :inertia obj-pp)) 1) (elt (cdr (assoc :inertia obj-pp)) 2) (cdr (assoc :kp obj-pp)) (cdr (assoc :kd obj-pp)) (cdr (assoc :mu1 obj-pp)) (cdr (assoc :mu2 obj-pp)) (elt (cdr (assoc :fdir1 obj-pp)) 0) (elt (cdr (assoc :fdir1 obj-pp)) 1) (elt (cdr (assoc :fdir1 obj-pp)) 2) cnt))
      ;; set object name to original urdf
      (unix::system 
       (format nil "sed -i -e 's@<objname>obj</objname>@<objname>obj~d</objname>@g' `rospack find hrpsys_gazebo_tutorials`/environment_models/box/model~d.urdf" cnt cnt))
      ;; set object name to original xacro

      (warn "generate ~a~%" (format nil "model~d.urdf" cnt))
      )
    ))

;; cupboard
(defun generate-cupboard
  (&key (error-ratio nil))

  (let* ((obj-pp-list ;; pysical parameter list
	  (list
	   ;; default
	   (list (cons :mass (get-value-with-error 5 error-ratio))
		 (cons :origin (float-vector 0 0 0.5))
		 (cons :inertia (float-vector 10 5.7411 0.788))
		 (cons :mu1 (get-value-with-error 0.1 error-ratio))
		 (cons :mu2 (get-value-with-error 0.1 error-ratio))
		 (cons :fdir1 (float-vector 1 0 0))
		 (cons :kp 1000000.0)
		 (cons :kd 100.0))
	   ;; high friction
	   (list (cons :mass (get-value-with-error 5 error-ratio))
		 (cons :origin (float-vector 0 0 0.5))
		 (cons :inertia (float-vector 10 5.7411 0.788))
		 (cons :mu1 (get-value-with-error 50.0 error-ratio))
		 (cons :mu2 (get-value-with-error 50.0 error-ratio))
		 (cons :fdir1 (float-vector 1 0 0))
		 (cons :kp 1000000.0)
		 (cons :kd 100.0))
	   ;; heavy
	   (list (cons :mass (get-value-with-error 7.5 error-ratio))
		 (cons :origin (float-vector 0 0 0.5))
		 (cons :inertia (float-vector 10 5.7411 0.788))
		 (cons :mu1 (get-value-with-error 0.1 error-ratio))
		 (cons :mu2 (get-value-with-error 0.1 error-ratio))
		 (cons :fdir1 (float-vector 1 0 0))
		 (cons :kp 1000000.0)
		 (cons :kd 100.0))
	   ;; heavy, high friction
	   (list (cons :mass (get-value-with-error 7.5 error-ratio))
		 (cons :origin (float-vector 0 0 0.5))
		 (cons :inertia (float-vector 10 5.7411 0.788))
		 (cons :mu1 (get-value-with-error 50.0 error-ratio))
		 (cons :mu2 (get-value-with-error 50.0 error-ratio))
		 (cons :fdir1 (float-vector 1 0 0))
		 (cons :kp 1000000.0)
		 (cons :kd 100.0))
	   ;; light
	   (list (cons :mass (get-value-with-error 2.5 error-ratio))
		 (cons :origin (float-vector 0 0 0.5))
		 (cons :inertia (float-vector 10 5.7411 0.788))
		 (cons :mu1 (get-value-with-error 0.1 error-ratio))
		 (cons :mu2 (get-value-with-error 0.1 error-ratio))
		 (cons :fdir1 (float-vector 1 0 0))
		 (cons :kp 1000000.0)
		 (cons :kd 100.0))
	   ;; very heavy
	   (list (cons :mass (get-value-with-error 10 error-ratio))
		 (cons :origin (float-vector 0 0 0.5))
		 (cons :inertia (float-vector 10 5.7411 0.788))
		 (cons :mu1 (get-value-with-error 0.1 error-ratio))
		 (cons :mu2 (get-value-with-error 0.1 error-ratio))
		 (cons :fdir1 (float-vector 1 0 0))
		 (cons :kp 1000000.0)
		 (cons :kd 100.0))
	   ))
	 (cnt 0))
    (dolist (obj-pp obj-pp-list)
      (incf cnt)
      ;; set physical parameter to original urdf
      (cond
       ((and (cdr (assoc :fdir1 obj-pp)) (cdr (assoc :kp obj-pp)) (cdr (assoc :kd obj-pp)))
	(unix::system 
	 (format nil "sed -e 's@  </link>@    <inertial>\\n      <mass value=\"~a\" />\\n      <origin xyz=\"~a ~a ~a\" rpy=\"0 0 0\"/>\\n      <inertia ixx=\"~a\" ixy=\"0\" ixz=\"0\" iyy=\"~a\" iyz=\"0\" izz=\"~a\"/>\\n    </inertial>\\n  </link>\\n  <gazebo reference=\"nil_link\">\\n    <kp>~a</kp>\\n    <kd>~a</kd>\\n    <mu1>~a</mu1>\\n    <mu2>~a</mu2>\\n    <fdir1>~a ~a ~a</fdir1>\\n  </gazebo>@g' `rospack find hrpsys_gazebo_tutorials`/environment_models/cupboard_wh/cupboard_body.urdf.in > `rospack find hrpsys_gazebo_tutorials`/environment_models/cupboard_wh/cupboard_body~d.urdf"
		 (cdr (assoc :mass obj-pp)) (elt (cdr (assoc :origin obj-pp)) 0) (elt (cdr (assoc :origin obj-pp)) 1) (elt (cdr (assoc :origin obj-pp)) 2) (elt (cdr (assoc :inertia obj-pp)) 0) (elt (cdr (assoc :inertia obj-pp)) 1) (elt (cdr (assoc :inertia obj-pp)) 2) (cdr (assoc :kp obj-pp)) (cdr (assoc :kd obj-pp)) (cdr (assoc :mu1 obj-pp)) (cdr (assoc :mu2 obj-pp)) (elt (cdr (assoc :fdir1 obj-pp)) 0) (elt (cdr (assoc :fdir1 obj-pp)) 1) (elt (cdr (assoc :fdir1 obj-pp)) 2) cnt))
	)
       (t
	(unix::system 
	 (format nil "sed -e 's@  </link>@    <inertial>\\n      <mass value=\"~a\" />\\n      <origin xyz=\"~a ~a ~a\" rpy=\"0 0 0\"/>\\n      <inertia ixx=\"~a\" ixy=\"0\" ixz=\"0\" iyy=\"~a\" iyz=\"0\" izz=\"~a\"/>\\n    </inertial>\\n  </link>\\n  <gazebo reference=\"nil_link\">\\n    <mu1>~a</mu1>\\n    <mu2>~a</mu2>\\n  </gazebo>@g' `rospack find hrpsys_gazebo_tutorials`/environment_models/cupboard_wh/cupboard_body.urdf.in > `rospack find hrpsys_gazebo_tutorials`/environment_models/cupboard_wh/cupboard_body~d.urdf"
		 (cdr (assoc :mass obj-pp)) (elt (cdr (assoc :origin obj-pp)) 0) (elt (cdr (assoc :origin obj-pp)) 1) (elt (cdr (assoc :origin obj-pp)) 2) (elt (cdr (assoc :inertia obj-pp)) 0) (elt (cdr (assoc :inertia obj-pp)) 1) (elt (cdr (assoc :inertia obj-pp)) 2) (cdr (assoc :mu1 obj-pp)) (cdr (assoc :mu2 obj-pp)) cnt))
	))
      ;; set object name to original urdf
      (unix::system 
       (format nil "sed -i -e 's@<objname>obj</objname>@<objname>obj~d</objname>@g' `rospack find hrpsys_gazebo_tutorials`/environment_models/cupboard_wh/cupboard_body~d.urdf" cnt cnt))
      ;; set object name to original xacro
      (unix::system 
       (format nil "sed -e 's@cupboard_wh/cupboard_body.urdf@cupboard_wh/cupboard_body~d.urdf@g' `rospack find hrpsys_gazebo_tutorials`/environment_models/cupboard_wh/model.urdf.xacro.in > `rospack find hrpsys_gazebo_tutorials`/environment_models/cupboard_wh/model~d.urdf.xacro" cnt cnt))
      ;; generate urdf by xacro
      (unix::system 
       (format nil "rosrun xacro xacro.py `rospack find hrpsys_gazebo_tutorials`/environment_models/cupboard_wh/model~d.urdf.xacro > `rospack find hrpsys_gazebo_tutorials`/environment_models/cupboard_wh/model~d.urdf" cnt cnt))
      (warn "generate ~a~%" (format nil "model~d.urdf" cnt))
      )
    ))

;; drum
(defun generate-drum
  ()

  (let* ((obj-pp-list ;; pysical parameter list
	  (list
	   (list (cons :mass 5)
		 (cons :origin (float-vector 2 0 0.2))
		 (cons :inertia (float-vector 0.1 0.1 0.1))
		 ;; (cons :inertia (float-vector 6.6875 5.7411 0.788))
		 (cons :mu1 0.1)
		 (cons :mu2 0.1))
	   (list (cons :mass 10)
		 (cons :origin (float-vector 2 0 0.2))
		 (cons :inertia (float-vector 0.5 0.5 0.5))
		 (cons :mu1 0.1)
		 (cons :mu2 0.1))
	   (list (cons :mass 20)
		 (cons :origin (float-vector 2 0 0.05))
		 (cons :inertia (float-vector 2.0 2.0 1.0))
		 (cons :mu1 0.1)
		 (cons :mu2 0.1))
	   (list (cons :mass 20)
		 (cons :origin (float-vector 2 0 0.5))
		 (cons :inertia (float-vector 2.0 2.0 1.0))
		 (cons :mu1 0.1)
		 (cons :mu2 0.1))
	   (list (cons :mass 20)
		 (cons :origin (float-vector 2 0 0.2))
		 (cons :inertia (float-vector 0.5 0.5 0.5))
		 (cons :mu1 0.3)
		 (cons :mu2 0.3)
		 (cons :fdir1 (float-vector 1 0 0))
		 (cons :kp 1000000.0)
		 (cons :kd 100.0))
	   (list (cons :mass 20)
		 (cons :origin (float-vector 2 0 0.2))
		 (cons :inertia (float-vector 0.1 0.1 0.1))
		 (cons :mu1 1.0)
		 (cons :mu2 1.0)
		 (cons :fdir1 (float-vector 1 0 0))
		 (cons :kp 1000000.0)
		 (cons :kd 100.0))))
	 (cnt 0))
    (dolist (obj-pp obj-pp-list)
      (incf cnt)
      ;; set physical parameter to original urdf
      (cond
       ((and (cdr (assoc :fdir1 obj-pp)) (cdr (assoc :kp obj-pp)) (cdr (assoc :kd obj-pp)))
	(unix::system 
	 (format nil "sed -e 's@  </link>@    <inertial>\\n      <mass value=\"~a\" />\\n      <origin xyz=\"~a ~a ~a\" rpy=\"0 0 0\"/>\\n      <inertia ixx=\"~a\" ixy=\"0\" ixz=\"0\" iyy=\"~a\" iyz=\"0\" izz=\"~a\"/>\\n    </inertial>\\n  </link>\\n  <gazebo reference=\"nil_link\">\\n    <kp>~a</kp>\\n    <kd>~a</kd>\\n    <mu1>~a</mu1>\\n    <mu2>~a</mu2>\\n    <fdir1>~a ~a ~a</fdir1>\\n  </gazebo>@g' `rospack find hrpsys_gazebo_tutorials`/environment_models/drum_wh/drum_body.urdf.in > `rospack find hrpsys_gazebo_tutorials`/environment_models/drum_wh/drum_body~d.urdf"
		 (cdr (assoc :mass obj-pp)) (elt (cdr (assoc :origin obj-pp)) 0) (elt (cdr (assoc :origin obj-pp)) 1) (elt (cdr (assoc :origin obj-pp)) 2) (elt (cdr (assoc :inertia obj-pp)) 0) (elt (cdr (assoc :inertia obj-pp)) 1) (elt (cdr (assoc :inertia obj-pp)) 2) (cdr (assoc :kp obj-pp)) (cdr (assoc :kd obj-pp)) (cdr (assoc :mu1 obj-pp)) (cdr (assoc :mu2 obj-pp)) (elt (cdr (assoc :fdir1 obj-pp)) 0) (elt (cdr (assoc :fdir1 obj-pp)) 1) (elt (cdr (assoc :fdir1 obj-pp)) 2) cnt))
	)
       (t
	(unix::system 
	 (format nil "sed -e 's@  </link>@    <inertial>\\n      <mass value=\"~a\" />\\n      <origin xyz=\"~a ~a ~a\" rpy=\"0 0 0\"/>\\n      <inertia ixx=\"~a\" ixy=\"0\" ixz=\"0\" iyy=\"~a\" iyz=\"0\" izz=\"~a\"/>\\n    </inertial>\\n  </link>\\n  <gazebo reference=\"nil_link\">\\n    <mu1>~a</mu1>\\n    <mu2>~a</mu2>\\n  </gazebo>@g' `rospack find hrpsys_gazebo_tutorials`/environment_models/drum_wh/drum_body.urdf.in > `rospack find hrpsys_gazebo_tutorials`/environment_models/drum_wh/drum_body~d.urdf"
		 (cdr (assoc :mass obj-pp)) (elt (cdr (assoc :origin obj-pp)) 0) (elt (cdr (assoc :origin obj-pp)) 1) (elt (cdr (assoc :origin obj-pp)) 2) (elt (cdr (assoc :inertia obj-pp)) 0) (elt (cdr (assoc :inertia obj-pp)) 1) (elt (cdr (assoc :inertia obj-pp)) 2) (cdr (assoc :mu1 obj-pp)) (cdr (assoc :mu2 obj-pp)) cnt))
	))
      ;; set object name to original urdf
      (unix::system 
       (format nil "sed -i -e 's@<objname>obj</objname>@<objname>obj~d</objname>@g' `rospack find hrpsys_gazebo_tutorials`/environment_models/drum_wh/drum_body~d.urdf" cnt cnt))
      ;; set object name to original xacro
      (unix::system 
       (format nil "sed -e 's@drum_wh/drum_body.urdf@drum_wh/drum_body~d.urdf@g' `rospack find hrpsys_gazebo_tutorials`/environment_models/drum_wh/model.urdf.xacro.in > `rospack find hrpsys_gazebo_tutorials`/environment_models/drum_wh/model~d.urdf.xacro" cnt cnt))
      ;; generate urdf by xacro
      (unix::system 
       (format nil "rosrun xacro xacro.py `rospack find hrpsys_gazebo_tutorials`/environment_models/drum_wh/model~d.urdf.xacro > `rospack find hrpsys_gazebo_tutorials`/environment_models/drum_wh/model~d.urdf" cnt cnt))
      (warn "generate ~a~%" (format nil "model~d.urdf" cnt))
      )
    ))

;; cart
(defun generate-cart
  (&key (error-ratio nil))

  (let* ((obj-pp-list ;; pysical parameter list
	  (list
	   ;; default
	   (list (cons :mass (get-value-with-error 5 error-ratio))
		 (cons :origin (float-vector 0 0 0.2))
		 (cons :inertia (float-vector 10 5.7411 0.788))
		 (cons :mu1 (get-value-with-error 0.1 error-ratio))
		 (cons :mu2 (get-value-with-error 0.1 error-ratio))
		 (cons :fdir1 (float-vector 1 0 0))
		 (cons :kp 1000000.0)
		 (cons :kd 100.0))
	   ;; high friction
	   (list (cons :mass (get-value-with-error 5 error-ratio))
		 (cons :origin (float-vector 0 0 0.2))
		 (cons :inertia (float-vector 10 5.7411 0.788))
		 (cons :mu1 (get-value-with-error 50.0 error-ratio))
		 (cons :mu2 (get-value-with-error 50.0 error-ratio))
		 (cons :fdir1 (float-vector 1 0 0))
		 (cons :kp 1000000.0)
		 (cons :kd 100.0))
	   ;; heavy
	   (list (cons :mass (get-value-with-error 7.5 error-ratio))
		 (cons :origin (float-vector 0 0 0.2))
		 (cons :inertia (float-vector 10 5.7411 0.788))
		 (cons :mu1 (get-value-with-error 0.1 error-ratio))
		 (cons :mu2 (get-value-with-error 0.1 error-ratio))
		 (cons :fdir1 (float-vector 1 0 0))
		 (cons :kp 1000000.0)
		 (cons :kd 100.0))
	   ;; heavy, high friction
	   (list (cons :mass (get-value-with-error 7.5 error-ratio))
		 (cons :origin (float-vector 0 0 0.2))
		 (cons :inertia (float-vector 10 5.7411 0.788))
		 (cons :mu1 (get-value-with-error 50.0 error-ratio))
		 (cons :mu2 (get-value-with-error 50.0 error-ratio))
		 (cons :fdir1 (float-vector 1 0 0))
		 (cons :kp 1000000.0)
		 (cons :kd 100.0))
	   ;; light
	   (list (cons :mass (get-value-with-error 2.5 error-ratio))
		 (cons :origin (float-vector 0 0 0.2))
		 (cons :inertia (float-vector 10 5.7411 0.788))
		 (cons :mu1 (get-value-with-error 0.1 error-ratio))
		 (cons :mu2 (get-value-with-error 0.1 error-ratio))
		 (cons :fdir1 (float-vector 1 0 0))
		 (cons :kp 1000000.0)
		 (cons :kd 100.0))
	   ;; very heavy
	   (list (cons :mass (get-value-with-error 10 error-ratio))
		 (cons :origin (float-vector 0 0 0.2))
		 (cons :inertia (float-vector 10 5.7411 0.788))
		 (cons :mu1 (get-value-with-error 0.1 error-ratio))
		 (cons :mu2 (get-value-with-error 0.1 error-ratio))
		 (cons :fdir1 (float-vector 1 0 0))
		 (cons :kp 1000000.0)
		 (cons :kd 100.0))
	   ))
	 (cnt 0))
    (dolist (obj-pp obj-pp-list)
      (incf cnt)
      ;; set physical parameter to original urdf
      (cond
       ((and (cdr (assoc :fdir1 obj-pp)) (cdr (assoc :kp obj-pp)) (cdr (assoc :kd obj-pp)))
	(unix::system 
	 (format nil "sed -e 's@  </link>@    <inertial>\\n      <mass value=\"~a\" />\\n      <origin xyz=\"~a ~a ~a\" rpy=\"0 0 0\"/>\\n      <inertia ixx=\"~a\" ixy=\"0\" ixz=\"0\" iyy=\"~a\" iyz=\"0\" izz=\"~a\"/>\\n    </inertial>\\n  </link>\\n  <gazebo reference=\"nil_link\">\\n    <kp>~a</kp>\\n    <kd>~a</kd>\\n    <mu1>~a</mu1>\\n    <mu2>~a</mu2>\\n    <fdir1>~a ~a ~a</fdir1>\\n  </gazebo>@g' `rospack find hrpsys_gazebo_tutorials`/environment_models/metal_small_cart_wh/metal_small_cart_body.urdf.in > `rospack find hrpsys_gazebo_tutorials`/environment_models/metal_small_cart_wh/metal_small_cart_body~d.urdf"
		 (cdr (assoc :mass obj-pp)) (elt (cdr (assoc :origin obj-pp)) 0) (elt (cdr (assoc :origin obj-pp)) 1) (elt (cdr (assoc :origin obj-pp)) 2) (elt (cdr (assoc :inertia obj-pp)) 0) (elt (cdr (assoc :inertia obj-pp)) 1) (elt (cdr (assoc :inertia obj-pp)) 2) (cdr (assoc :kp obj-pp)) (cdr (assoc :kd obj-pp)) (cdr (assoc :mu1 obj-pp)) (cdr (assoc :mu2 obj-pp)) (elt (cdr (assoc :fdir1 obj-pp)) 0) (elt (cdr (assoc :fdir1 obj-pp)) 1) (elt (cdr (assoc :fdir1 obj-pp)) 2) cnt))
	)
       (t
	(unix::system 
	 (format nil "sed -e 's@  </link>@    <inertial>\\n      <mass value=\"~a\" />\\n      <origin xyz=\"~a ~a ~a\" rpy=\"0 0 0\"/>\\n      <inertia ixx=\"~a\" ixy=\"0\" ixz=\"0\" iyy=\"~a\" iyz=\"0\" izz=\"~a\"/>\\n    </inertial>\\n  </link>\\n  <gazebo reference=\"nil_link\">\\n    <mu1>~a</mu1>\\n    <mu2>~a</mu2>\\n  </gazebo>@g' `rospack find hrpsys_gazebo_tutorials`/environment_models/metal_small_cart_wh/metal_small_cart_body.urdf.in > `rospack find hrpsys_gazebo_tutorials`/environment_models/metal_small_cart_wh/metal_small_cart_body~d.urdf"
		 (cdr (assoc :mass obj-pp)) (elt (cdr (assoc :origin obj-pp)) 0) (elt (cdr (assoc :origin obj-pp)) 1) (elt (cdr (assoc :origin obj-pp)) 2) (elt (cdr (assoc :inertia obj-pp)) 0) (elt (cdr (assoc :inertia obj-pp)) 1) (elt (cdr (assoc :inertia obj-pp)) 2) (cdr (assoc :mu1 obj-pp)) (cdr (assoc :mu2 obj-pp)) cnt))
	))
      ;; set object name to original urdf
      (unix::system 
       (format nil "sed -i -e 's@<objname>obj</objname>@<objname>obj~d</objname>@g' `rospack find hrpsys_gazebo_tutorials`/environment_models/metal_small_cart_wh/metal_small_cart_body~d.urdf" cnt cnt))
      ;; set object name to original xacro
      (unix::system 
       (format nil "sed -e 's@metal_small_cart_wh/metal_small_cart_body.urdf@metal_small_cart_wh/metal_small_cart_body~d.urdf@g' `rospack find hrpsys_gazebo_tutorials`/environment_models/metal_small_cart_wh/model.urdf.xacro.in > `rospack find hrpsys_gazebo_tutorials`/environment_models/metal_small_cart_wh/model~d.urdf.xacro" cnt cnt))
      ;; generate urdf by xacro
      (unix::system 
       (format nil "rosrun xacro xacro.py `rospack find hrpsys_gazebo_tutorials`/environment_models/metal_small_cart_wh/model~d.urdf.xacro > `rospack find hrpsys_gazebo_tutorials`/environment_models/metal_small_cart_wh/model~d.urdf" cnt cnt))
      (warn "generate ~a~%" (format nil "model~d.urdf" cnt))
      )
    ))


;; util function  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; error
(defun get-value-with-error
  (value &optional (error-ratio nil)) ;; add error of plus minus error-ratio [%]

  (unless error-ratio
    (return-from get-value-with-error value))
  (let* ((error-range (* value (/ error-ratio 100.0)))
	 (error-value (- (random (* error-range)) error-range))
	 (value-with-error (+ value error-value)))
    value-with-error))

