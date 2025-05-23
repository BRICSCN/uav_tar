;; Auto-generated. Do not edit!


(when (boundp 'yolov5_ros_msgs::BoundingBoxes)
  (if (not (find-package "YOLOV5_ROS_MSGS"))
    (make-package "YOLOV5_ROS_MSGS"))
  (shadow 'BoundingBoxes (find-package "YOLOV5_ROS_MSGS")))
(unless (find-package "YOLOV5_ROS_MSGS::BOUNDINGBOXES")
  (make-package "YOLOV5_ROS_MSGS::BOUNDINGBOXES"))

(in-package "ROS")
;;//! \htmlinclude BoundingBoxes.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass yolov5_ros_msgs::BoundingBoxes
  :super ros::object
  :slots (_header _image_header _bounding_boxes ))

(defmethod yolov5_ros_msgs::BoundingBoxes
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:image_header __image_header) (instance std_msgs::Header :init))
    ((:bounding_boxes __bounding_boxes) (let (r) (dotimes (i 0) (push (instance yolov5_ros_msgs::BoundingBox :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _image_header __image_header)
   (setq _bounding_boxes __bounding_boxes)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:image_header
   (&rest __image_header)
   (if (keywordp (car __image_header))
       (send* _image_header __image_header)
     (progn
       (if __image_header (setq _image_header (car __image_header)))
       _image_header)))
  (:bounding_boxes
   (&rest __bounding_boxes)
   (if (keywordp (car __bounding_boxes))
       (send* _bounding_boxes __bounding_boxes)
     (progn
       (if __bounding_boxes (setq _bounding_boxes (car __bounding_boxes)))
       _bounding_boxes)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; std_msgs/Header _image_header
    (send _image_header :serialization-length)
    ;; yolov5_ros_msgs/BoundingBox[] _bounding_boxes
    (apply #'+ (send-all _bounding_boxes :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; std_msgs/Header _image_header
       (send _image_header :serialize s)
     ;; yolov5_ros_msgs/BoundingBox[] _bounding_boxes
     (write-long (length _bounding_boxes) s)
     (dolist (elem _bounding_boxes)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; std_msgs/Header _image_header
     (send _image_header :deserialize buf ptr-) (incf ptr- (send _image_header :serialization-length))
   ;; yolov5_ros_msgs/BoundingBox[] _bounding_boxes
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _bounding_boxes (let (r) (dotimes (i n) (push (instance yolov5_ros_msgs::BoundingBox :init) r)) r))
     (dolist (elem- _bounding_boxes)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get yolov5_ros_msgs::BoundingBoxes :md5sum-) "9e020ee6562f07774a9287a794924bfc")
(setf (get yolov5_ros_msgs::BoundingBoxes :datatype-) "yolov5_ros_msgs/BoundingBoxes")
(setf (get yolov5_ros_msgs::BoundingBoxes :definition-)
      "Header header
Header image_header
BoundingBox[] bounding_boxes

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: yolov5_ros_msgs/BoundingBox
float64 probability
int64 xmin
int64 ymin
int64 xmax
int64 ymax
int16 num
string Class
int64 id

")



(provide :yolov5_ros_msgs/BoundingBoxes "9e020ee6562f07774a9287a794924bfc")


