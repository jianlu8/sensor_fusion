
# 四个角点坐标按顺时针方向，cm
float32 x1
float32 y1
float32 x2
float32 y2
float32 x3
float32 y3
float32 x4
float32 y4

# 高度, cm
float32 zmin
float32 zmax

# 相对车速, m/s
float32 speed

# deg
float32 speed_direction

uint32 id
string label

#与ego车道的对应关系
# 0:障碍物位于自车所在道（若自车变道，以前轴中心越线为切换点）
#1、2…：分别位于左1、2…车道
#-1、-2…：分别位于右1、2…车道
int32 relative_lane
#与HD MAP车道的对应关系
int32 global_lane

# , m
float32 global_x
float32 global_y

#以上坐标信息均在车辆坐标系下
#车辆坐标系  ---  后轴中心坐标系，车头方向Y，车头右侧X
