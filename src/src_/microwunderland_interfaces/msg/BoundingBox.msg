# used to send the resoults from a object detector that outputs bounding boxes
Vector2D vec_a      # if minMax formate vec_a = min; if centerSize format vec_a = center
Vector2D vec_b      # if minMax formate vec_a = max; if centerSize format vec_a = size
uint8 format        # 0 equlas minMax; 1 equlas centerSize
string label_name   # name of the detected object