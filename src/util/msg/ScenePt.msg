# 静动态场景

uint32 point_id     # 中心线上的形点ID

float64 x               # m,中心线上的形点gauss x
float64 y               # m,中心线上的形点gauss y
float64 heading         # deg, east=0, north=90

uint32 lane_idx         # 挂载到对应车道的车道索引
float64 s               # m,中心线上的形点累计弧长s
float64 d               # m,中心线上的形点横向距离d

uint8 type              # 场景类型

uint8 StationType=0
uint8 ParkingType=1

