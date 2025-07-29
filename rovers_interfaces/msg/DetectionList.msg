# A list of object detections from a single image frame
std_msgs/Header header   # Contains timestamp and frame_id from the original image
gemini_detector/Detection[] detections # An array of detected objects
