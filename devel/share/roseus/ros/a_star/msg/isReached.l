;; Auto-generated. Do not edit!


(when (boundp 'a_star::isReached)
  (if (not (find-package "A_STAR"))
    (make-package "A_STAR"))
  (shadow 'isReached (find-package "A_STAR")))
(unless (find-package "A_STAR::ISREACHED")
  (make-package "A_STAR::ISREACHED"))

(in-package "ROS")
;;//! \htmlinclude isReached.msg.html


(defclass a_star::isReached
  :super ros::object
  :slots (_Reached ))

(defmethod a_star::isReached
  (:init
   (&key
    ((:Reached __Reached) nil)
    )
   (send-super :init)
   (setq _Reached __Reached)
   self)
  (:Reached
   (&optional __Reached)
   (if __Reached (setq _Reached __Reached)) _Reached)
  (:serialization-length
   ()
   (+
    ;; bool _Reached
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _Reached
       (if _Reached (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _Reached
     (setq _Reached (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get a_star::isReached :md5sum-) "1c2e8beeec8a67b9955f2207bd7a9654")
(setf (get a_star::isReached :datatype-) "a_star/isReached")
(setf (get a_star::isReached :definition-)
      "bool Reached
")



(provide :a_star/isReached "1c2e8beeec8a67b9955f2207bd7a9654")


