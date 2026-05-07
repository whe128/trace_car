int8 STRAIGHT=0
int8 TURN = 1
int8 CROSS = 2
int8 STOP = 3
int8 BLACK = 4


# ===== Far CCD =====
uint8[128] far_img
int8 far_left_edge
int8 far_right_edge
int8 far_center
int8 far_road_width
uint8 far_avg_brightness


# ===== Close CCD =====
uint8[128] close_img
int8 close_left_edge
int8 close_right_edge
int8 close_center
int8 close_road_width
uint8 close_avg_brightness
