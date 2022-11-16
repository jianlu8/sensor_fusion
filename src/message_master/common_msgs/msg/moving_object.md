由于msg文件不宜更改，各变量的单位和定义均有可能变化，实际定义以md文件为准
#relative position in base_link, m
float32 x1
float32 y1
float32 x2
float32 y2
float32 x3
float32 y3
float32 x4
float32 y4

# height in base_link, m
float32 zmin

float32 zmax

# relative speed, m/s
float32 speed

# deg, OX=0, OY=90
float32 speed_direction


uint32 id
string label

# center point coordinate in localpose frame, m
float32 global_x
float32 global_y
