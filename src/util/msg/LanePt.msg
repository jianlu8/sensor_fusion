# 车道点


uint32 point_id     # 中心线上的形点ID

float64 x               # m       中心线上的形点gauss x
float64 y               # m       中心线上的形点gauss y
float64 s               # m       中心线上的形点累计弧长s
float64 heading         # deg, east=0, north=90

float64 speed           # m/s, 限速值

float64 upper_boundary # 形点距离左道路边界, m, left > 0
float64 lower_boundary # 形点距离右道路边界, m, right < 0

float64 planning_upper_limit # 形点距离左车道边界,m, left > 0
float64 planning_lower_limit # 形点距离右车道边界, m, right < 0

uint8 junc_status # 形点的路口状态
uint8 left_traverse_flag # 形点向左跨越边界的能力
uint8 right_traverse_flag # 形点向右跨越边界的能力

uint8 outlier_junc=0 # 路口之外
uint8 incoming_junc=1 # 即将进入路口
uint8 inner_junc=2 # 在路口内
uint8 outgoing_junc=3 # 刚刚驶离路口

uint8 traverse_unknown=0 # 未知的跨越属性
uint8 traverse_broken=1 # 虚线
uint8 traverse_solid=2 # 实线
uint8 traverse_curb=3 # 物理边界