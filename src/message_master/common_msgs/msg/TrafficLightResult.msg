# This message defines meta information for a traffic light status
Header header
int32 light_id                                  #红绿灯ID
float64 probability                             #识别结果质信度
int32 recognition_result                        #红绿灯识别结果, RED= 0,YELLOW= 1,GREEN= 2,UNKNOWN= 3
int32 remainder_time                            #红绿灯剩余时间(单位:s)
geometry_msgs/Vector3 pose                      #局部坐标系下红绿灯位置
int32 turn_type                                 #红绿灯转向类型, 直行= 0，左转= 1，右转= 2，未知= 3



