# Custom message with one inference result details.
#
# This message is used to communicate the inference results.
int32 class_label   # Identifier for a particular class.
float32 class_prob  # Predicted probability value for the class.
# Bounding box localization information.
float32 x_min
float32 y_min
float32 x_max
float32 y_max