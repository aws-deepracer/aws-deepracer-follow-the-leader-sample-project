# Custom message with an array of inference result and corresponding state details.
#
# This message is used to communicate the inference results and the corresponding
# state input (images) on which the inference was run on. 
InferResults[] results      # Array of inference results.
sensor_msgs/Image[] images  # Array of input images on which the inference was run. 