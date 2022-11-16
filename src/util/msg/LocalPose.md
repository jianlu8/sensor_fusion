由于msg文件不宜更改，各变量的单位和定义均有可能变化，实际定义以md文件为准

#LocalPose
Header header
int32 	plat_id				#标识此消息对应的平台编号

int32 	error_code   		#错误标识码

int32 	gps_week			#GPS Week
int32 	gps_millisecond     #GPS millionsecond in a week ,单位：ms

float64 dr_x           		#航迹推算车辆位置,单位：cm
float64 dr_y
float64 dr_z

float64 dr_heading     		#车体姿态,单位:1.0 degree	, 航向角OX方向为零度,逆时针方向为正,0-360度	
float64 dr_roll				
float64 dr_pitch		


int32 	vehicle_speed   	#车速：cm/s  负值表示倒车

#imu三轴陀螺速度 
float64 rot_x       		#x轴方向右手定则,四指方向为正,单位0.01 deg/s
float64 rot_y				#y轴方向右手定则,四指方向为正,单位0.01 deg/s
float64 rot_z				#z轴方向右手定则,四指方向为正,单位0.01 deg/s

#imu三轴加速度 
float64 acc_x				#x轴方向加速度(车体正右方)	单位0.01m/s^2
float64 acc_y				#y轴方向加速度(车体正前方)	单位0.01m/s^2
float64 acc_z				#z轴方向加速度(车体正上方)	单位0.01m/s^2
