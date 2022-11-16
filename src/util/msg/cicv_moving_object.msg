std_msgs/Header  header
uint32 id                                  #目标ID
string label                             #目标类型
float32 probability              #障碍物存在概率(取值范围0-1)
float32 label_probability #障碍物类型概率(取值范围0-1)
uint32 motion_status        #障碍物运动状态
float32 ttc                                #TTC
float32 variance                    #方差
string data_source              #目标数据来源，0：融合结果，1：激光雷达，2：视觉，3：毫米波雷达，4：超声波雷达
float32 x1                #相对坐标系下的左前坐标（单位cm）
float32 y1                #相对坐标系下的左前坐标（单位cm）
float32 x2                #相对坐标系下的右前坐标（单位cm）
float32 y2                #相对坐标系下的右前坐标（单位cm）
float32 x3                #相对坐标系下的右后坐标（单位cm）
float32 y3                #相对坐标系下的右后坐标（单位cm）
float32 x4                #相对坐标系下的左后坐标（单位cm）
float32 y4                #相对坐标系下的左后坐标（单位cm）
float32 min_z                         #最小高度
float32 max_z                         #最大高度
geometry_msgs/Pose              pose                                  #相对坐标系下的障碍物中心点坐标
geometry_msgs/Vector3        dimensions                    #相对坐标系下的障碍物尺寸
geometry_msgs/Twist              velocity                           #相对坐标系下的障碍物速度信息(速度矢量)
geometry_msgs/Twist              acceleration                  #相对坐标系下的障碍物加速度信息(速度矢量)
geometry_msgs/PolygonStamped    convex_hull    #障碍物多边形框(暂时不用赋值)
float32 speed                         #相对坐标系下的速度(sor文档需求)
float32 speed_direction    #相对坐标系下 的速度方向(sor文档需求)
float32 global_x                    #全局下的x坐标
float32 global_y                    #全局下的y坐标
int32 lane                                 #sor里面需求
#与ego车道的对应关系
# 0:障碍物位于自车所在道（若自车变道，以前轴中心越线为切换点）
#1、2…：分别位于左1、2…车道
#-1、-2…：分别位于右1、2…车道
int32 relative_lane
#与HD MAP车道的对应关系
int32 global_lane
