; Auto-generated. Do not edit!


(cl:in-package a_star-msg)


;//! \htmlinclude isReached.msg.html

(cl:defclass <isReached> (roslisp-msg-protocol:ros-message)
  ((Reached
    :reader Reached
    :initarg :Reached
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass isReached (<isReached>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <isReached>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'isReached)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name a_star-msg:<isReached> is deprecated: use a_star-msg:isReached instead.")))

(cl:ensure-generic-function 'Reached-val :lambda-list '(m))
(cl:defmethod Reached-val ((m <isReached>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a_star-msg:Reached-val is deprecated.  Use a_star-msg:Reached instead.")
  (Reached m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <isReached>) ostream)
  "Serializes a message object of type '<isReached>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'Reached) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <isReached>) istream)
  "Deserializes a message object of type '<isReached>"
    (cl:setf (cl:slot-value msg 'Reached) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<isReached>)))
  "Returns string type for a message object of type '<isReached>"
  "a_star/isReached")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'isReached)))
  "Returns string type for a message object of type 'isReached"
  "a_star/isReached")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<isReached>)))
  "Returns md5sum for a message object of type '<isReached>"
  "1c2e8beeec8a67b9955f2207bd7a9654")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'isReached)))
  "Returns md5sum for a message object of type 'isReached"
  "1c2e8beeec8a67b9955f2207bd7a9654")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<isReached>)))
  "Returns full string definition for message of type '<isReached>"
  (cl:format cl:nil "bool Reached~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'isReached)))
  "Returns full string definition for message of type 'isReached"
  (cl:format cl:nil "bool Reached~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <isReached>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <isReached>))
  "Converts a ROS message object to a list"
  (cl:list 'isReached
    (cl:cons ':Reached (Reached msg))
))
