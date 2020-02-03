# Results of signs on the lane

#enum
uint8 SIGNTYPE_SPEED_LIMIT = 0
uint8 SIGNTYPE_SPEED_LIMIT_END = 1
uint8 SIGNTYPE_TURN_LEFT = 2
uint8 SIGNTYPE_TURN_RIGHT = 3
uint8 SIGNTYPE_ZEBRA_CROSSING = 4
uint8 SIGNTYPE_BARRED_AREA = 5
uint8 SIGNTYPE_PEDESTRIAN_ISLAND = 6

uint8 signType
int64 value		# Holds speed limit value. For other signs it's value will be -1
float32 width
float32 height
ottocar_msgs/Point2D[4] boundingBox
