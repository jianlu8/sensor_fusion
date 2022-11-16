Header header
#VCU  
float32 AccPedalPos                  # 加速踏板位置
float32 ActualAccPedal               # 实际加速踏板开度
uint8   AutoDriveStatus              # 自动驾驶功能状态
uint8   AccPedalEnable               # 油门踏板干预使能
uint8   BrakePedalSignal             # 制动踏板开关信号
uint8   AccPedalFaultFlag            # 加速踏板信号异常标志
uint8   GearPos                      # 挡位物理位置
uint8   GearActual                   # 实际档位
uint8   ShiftLeverPosValidFlag       # 换挡杆位置是否有效信号

#VCU5
uint8 VCU_DrivingPermit
uint8 VCU_Gear

#EPB
uint8 EPBSystemStateIndicationReq  # EPB状态灯显示
uint8 EPBStatus                    # EPB状态

#ESC
uint16  FLWheelPulseCounter          # 左前轮速脉冲信号
uint16  FRWheelPulseCounter          # 右前轮速脉冲信号
uint16  RLWheelPulseCounter          # 左后轮速脉冲信号
uint16  RRWheelPulseCounter          # 右后轮速脉冲信号
uint8   FLWheelPulseCounterValidity  # 左前轮速脉冲信号有效标志
uint8   FRWheelPulseCounterValidity  # 右前轮速脉冲信号有效标志
float32 LongitudinalAcceleration     # 纵向加速度
uint8   RLWheelPulseCounterValidity  # 左后轮速脉冲信号有效标志
uint8   RRWheelPulseCounterValidity  # 右后轮速脉冲信号有效标志
uint8   LongitudinalAccelerationFlag # 纵向加速度标识

float32 YawRate             # 横摆角速度
uint8   YawRateValid        # 横摆角速度有效性
float32 LateralAcceleration # 横向加速度信号
uint8   AyvSensorValid      # 横向加速度有效性
uint8   StateAct            # 制动系统工作状态反馈

#EPS
uint8   DriverSteeringInterferenceDetected                      # 驾驶员干预方向盘检测
uint8   DriverSteeringInterferenceDetectedValidity              # 驾驶员干预方向盘检测有效性
uint8   ElectricalPowerSteeringAvailablityStatus                # EPS可控状态
uint8   ElectricalPowerSteeringAvailablityStatusProtectionValue # EPS可用状态保护算法值
float32 SteeringTorqueAct                                       # 方向盘力矩反馈

uint8 WarningLampState             # EPS警告灯状态

float32 SteeringAngle                # 转向角信号
uint16  SteeringRotSpd               # 转向角速度

float32 EPS_Ang_Stat_SteeringAngle
uint16 EPS_Ang_Stat_SteeringRotSpd
uint8 EPS_Dig_Stat_EPSAvailablity
uint8 EPS2_Rollingcount
uint8 EPS2_RollingcountChecksum

#GW_PCL1
uint8 PCG_Work_Sts
uint8 PCG_Error_Mode
uint8 PCG_Anti_Pinch_Mode
uint8 PCG_Open_Count

#ESC3

uint8 EBD_Fault
uint8 ABS_Fault
uint8 ABS_work_Flag
uint8 Wheel_Speed_Fault
uint8 ABS_Function
uint8 ABS_Fault_Lamp
uint8 EBD_Fault_Lamp
float32 Wheel_Speed_ABS
int32 VehicleSpeed
uint8 ESC_EnmergencySwtich
uint8 ESC_Low_Liguid_Level_Warning


#BCM
uint8 BCM_Flg_Stat_LeftTurningLamp
uint8 BCM_Flg_Stat_RightTurningLamp
uint8 BCM_Flg_Stat_HighBeam
uint8 BCM_Flg_Stat_LowBeam
uint8 BCM_Flg_Stat_RearFogLamp
uint8 BCM_Flg_Stat_DangerAlarmLamp
uint8 BCM_Flg_Stat_BrakeLamp
uint8 BCM_Flg_Stat_BackupLamp
uint8 BCM_Flg_Stat_SeatSensor1
uint8 BCM_Flg_Stat_SeatSensor2
uint8 BCM_Flg_Stat_SeatSensor3
uint8 BCM_Flg_Stat_SeatSensor4
uint8 BCM_Flg_Stat_SeatSensor5
uint8 BCM_Flg_Stat_SeatSensor6
uint8 BCM_Flg_Stat_BeltsSensor1
uint8 BCM_Flg_Stat_BeltsSensor2
uint8 BCM_Flg_Stat_BeltsSensor3
uint8 BCM_Flg_Stat_BeltsSensor4
uint8 BCM_Flg_Stat_BeltsSensor5
uint8 BCM_Flg_Stat_BeltsSensor6
uint8 BCM_FootStepSwitchSt
uint8 BCM_LetfligthFaultSt
uint8 BCM_RightligthFaultSt
uint8 BCM_DoorLockSt
uint8 BCM_IgnitionSt
uint8 BCM_DoorFaultSt
uint8 BCM_DoorSwitchSt
uint8 BCM_DoorPowerEn
uint8 BCM_IGONEn
uint8 BCM_HarFlashSwitchrSt
uint8 BCM_DoorOpenSwitchSt
uint8 BCM_DoorOpenOrderIN
uint8 BCM_DoorCloseOrderIN
uint8 BCM_DoorOpenOrderOUT
uint8 BCM_DoorCloseOrderOUT
uint8 BCM_PedalExpansionReq
uint8 BCM_OUTDoorOpenSwitchSt
uint8 BCM_ACCEnable
uint8 BCM_DRLRuningSt
uint8 BCM_InteriorLightingSt
uint8 BCM_SpeakerOutputSt
uint8 BCM_DemisterStatus
uint8 BCM_CentralLockUnlockSt
uint8 BCM_CentralLocklockSt
uint8 BCM_RLTurnlightFBSt
uint8 BCM_FLTurnlightFBSt
uint8 BCM_RRTurnlightFbSt
uint8 BCM_FRTurnlightFBSt
uint8 BCM_AutoLightSt

#GW_PCGL2
uint8 PCG_Lock1_Sts
uint8 PCG_Lock2_Sts
uint8 PCG_Gesture_SW
uint8 Gete_Work_Mode

#BMS
uint8 BMS_SOCCalculate
uint8 BMS_Alarm

#ESC4
float32 ESC_FrontLeftWHeelSpd
uint8 ESC_FLWHeelSpdFaultSignal
float32 ESC_FrontRightWHeelSpd
uint8 ESC_FRWHeelSpdFaultSignal
float32 ESC_RearLeftWheelSpd
uint8 ESC_RLWHeelSpdFaultSignal
float32 ESC_RearRightWheelSpd
uint8 ESC_RRWHeelSpdFaultSignal

#GW_MCU1
float32 VCU_TorqueDemand
uint8 VCU_TorqueRequestCRC
uint8 VCU_TorqueRequestCLK

#GW_MCU
int8 MCU_InverterTemp
int8 MCU_ElecMachineTemp
int16 MCU_Torqueestimation
int16 MCU_ElecMachineSpeed

#GW_PCGR1
uint8 PCG_Work_Sts_R1
uint8 PCG_Error_Mode_R1
uint8 PCG_Anti_Pinch_Mode_R1
uint8 PCG_Open_Count_R1

#GW_PCGR2
uint8 PCG_Lock1_Sts_R2
uint8 PCG_Lock2_Sts_R2
uint8 Gete_Work_Mode_R2



#eBooster
uint8 eBooster_eBoosterRdySt	#
uint8 eBooster_WorkingMode
uint8 eBooster_Brake_fehler_status	
float32 eBooster_d_Pedal
uint8 eBooster_Braking_Led_req
float32 eBooster_Brake_Pressure
uint8 eBooster_RollingCount_eBooster1
uint16 eBooster_CheckSum_eBooster1

# HAD
uint8 global_flag_waiting_rcu_feedback #等待RCU反馈标志
uint8 global_flag_waiting_rcu_control #等待RCU控制标志
uint8 global_emergency_stop_flag #紧急制动标志
uint8 autodrive_en #手柄使能状态
uint8 global_control_algo_flag #控制算法执行标志

uint8 global_flag_send_arp_request #发送ARP请求标志
uint8 global_canMsgTX_Fault #发送上层错误数据标志

uint8 global_vehicleAutoDriveStatus #自动驾驶状态
uint8 global_firstEnableAutoDriveOrRemoteDrive #首次使能自动驾驶or远程驾驶切换标志

uint32 global_time_waiting_rcu_feedback_ms 
uint32 global_time_waiting_rcu_control_ms 
uint32 global_connect_tx2_timeout #上下层通信超时
uint32 global_rcu_219_timeout