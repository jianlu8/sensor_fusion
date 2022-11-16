
#“限速标识”，“解除限速标识”，“禁止超车标识”，“解除禁止超车标识”，“施工指示标志”，“道路引导线”
string label                                    #标识类型
geometry_msgs/Vector3 pose                      #标识标志在局部坐标系下位置 
int32 data                                      #交通标识标志结果
#label="禁止超车标识"                            #是否允许超车,data= true:允许，data= false：不许许
#label="道路引导线"                              #道路转向引导线转向类型，data = 0:直行，data = 1:左转，data = 2:右转，data = 2:未知
  


