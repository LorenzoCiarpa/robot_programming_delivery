;; Auto-generated. Do not edit!


(when (boundp 'mrsim::rodom)
  (if (not (find-package "MRSIM"))
    (make-package "MRSIM"))
  (shadow 'rodom (find-package "MRSIM")))
(unless (find-package "MRSIM::RODOM")
  (make-package "MRSIM::RODOM"))

(in-package "ROS")
;;//! \htmlinclude rodom.msg.html


(defclass mrsim::rodom
  :super ros::object
  :slots (_x _y _theta ))

(defmethod mrsim::rodom
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:theta __theta) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _theta (float __theta))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:theta
   (&optional __theta)
   (if __theta (setq _theta __theta)) _theta)
  (:serialization-length
   ()
   (+
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _theta
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _theta
       (sys::poke _theta (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _theta
     (setq _theta (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get mrsim::rodom :md5sum-) "938fa65709584ad8e77d238529be13b8")
(setf (get mrsim::rodom :datatype-) "mrsim/rodom")
(setf (get mrsim::rodom :definition-)
      "float64 x
float64 y
float64 theta
")



(provide :mrsim/rodom "938fa65709584ad8e77d238529be13b8")


