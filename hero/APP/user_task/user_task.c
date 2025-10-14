/**
  ****************************RM Warrior 2023****************************
  * @file       start_task.c/h
  * @brief      一个普通的心跳程序，如果程序没有问题，蓝灯以1Hz的频率闪烁
  *             同时也用来发送相关数值到上位机调参。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2023/2/         pxx              ......
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************RM Warrior 2023****************************
  */

#include "User_Task.h"
#include "main.h"
#include "stdio.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "led.h"
#include "adc.h"

//#include "Detect_Task.h"
#include "INS_Task.h"
#include "gimbal_task.h"
#include "chassis_remote_control.h"
#include "gimbal_behaviour.h"
#include "chassis_behaviour.h"
#include "ROS_Receive.h"
//#define user_is_error() toe_is_error(errorListLength)
#include "shoot.h"

#include "voltage_task.h"
#include "Kalman_Filter.h"
#include "uart1.h"
#include "bluetooth.h"
#include "referee.h"
#include "relays.h"

#include "RM_Cilent_UI.h"
#include "CAN_Receive.h"
#include "string.h"
#include "math.h"
#include "shoot.h"
#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t UserTaskStack;
#endif

//姿态角 单位 度
fp32 angle_degree[3] = {0.0f, 0.0f, 0.0f};
const Gimbal_Control_t* local_gimbal_control;
const chassis_move_t* local_chassis_move;
KalmanInfo Power_KalmanInfo_Structure;

extern int8_t temp_set;
const shoot_control_t* local_shoot_control;

fp32 local_power = 0.0f, local_buffer = 0.0f;

fp32 Power_Calc(void);
Graph_Data G1,G2,G3,G4,G5,G6,G7,G8,G9,G10,G11;
String_Data CH_SHOOT;
String_Data CH_AUTO;
String_Data CH_FLRB;
String_Data CAP;
String_Data GYRO;
String_Data CH_COLLEGE;
Float_Data CAP_BAT;
float cap_vol=0.0;
char auto_arr[4]="AUTO";
char shoot_arr[5]="shoot";
char flrb_arr[4]="FRBL";
char cap_arr[3]="CAP";
char gyro_arr[4]="GYRO";
char college_arr[5]="PRIME";
void UserTask(void *pvParameters)
{	
		/**************************自定义UI界面——START************************************/
	  /**************************静止不变的信息放在while循环之外************************************/
		/**************************每refresh7张图片需要taskdelay 100ms保证所有图形被正确加载************************************/

		memset(&G1,0,sizeof(G1));//中心垂线
		memset(&G2,0,sizeof(G2));//上击打线
		memset(&G3,0,sizeof(G3));//中心水平线
		memset(&G4,0,sizeof(G4));//枪管轴心线
		memset(&G5,0,sizeof(G5));//下击打线
		memset(&G6,0,sizeof(G6));//远距离击打线
		memset(&G7,0,sizeof(G7));//摩擦轮状态
		memset(&CH_SHOOT,0,sizeof(CH_SHOOT));//摩擦轮标识
		memset(&G8,0,sizeof(G8));//前装甲板状态
		memset(&G9,0,sizeof(G9));//左装甲板状态
		memset(&G10,0,sizeof(G10));//右装甲板状态
		memset(&G11,0,sizeof(G11));//后装甲板状态
		memset(&CH_FLRB,0,sizeof(CH_FLRB));//装甲板标识
		memset(&CH_COLLEGE,0,sizeof(CH_COLLEGE));//LOGO标识
		memset(&CAP,0,sizeof(CAP));//超电标识


//    Line_Draw(&G1,"091",UI_Graph_ADD,9,UI_Color_Purplish_red,1,960,330,960,620);
//		Line_Draw(&G2,"092",UI_Graph_ADD,9,UI_Color_Purplish_red,1,880,580,1040,580);
//		Line_Draw(&G3,"093",UI_Graph_ADD,9,UI_Color_Purplish_red,1,800,540,1120,540);
//		Line_Draw(&G4,"094",UI_Graph_ADD,9,UI_Color_Purplish_red,1,880,500,1040,500);
//		UI_ReFresh(7,G1,G2,G3,G4,G5,G6,G7);                          //绘制图形	
//Float_Draw(&CAP_BAT,"095",UI_Graph_ADD,9,UI_Color_Purplish_red,1,2,2,100,860,cap_vol);//超电电量
		Line_Draw(&G6,"096",UI_Graph_ADD,9,UI_Color_Purplish_red,2,571,470,1345,470);//瞄准线
		Circle_Draw(&G8,"098",UI_Graph_ADD,9,UI_Color_Orange,15,230,820,15);//"CAP"右侧圆圈1
		Circle_Draw(&G1,"091",UI_Graph_ADD,9,UI_Color_Orange,15,280,820,15);//"CAP"右侧圆圈2
		Circle_Draw(&G2,"092",UI_Graph_ADD,9,UI_Color_Orange,15,330,820,15);//"CAP"右侧圆圈3
		Circle_Draw(&G3,"093",UI_Graph_ADD,9,UI_Color_Orange,15,380,820,15);//"CAP"右侧圆圈4
		UI_ReFresh(5,G1,G2,G3,G6,G8);                         							//推送上述元素	
		vTaskDelay(100);
		/**************************固定图形展示************************************/
		Rectangle_Draw(&G8,"081",UI_Graph_ADD,8,UI_Color_Purplish_red,3,900,460,1020,400);//”矩形瞄准框“
		UI_ReFresh(1,G8);//G7缺省  
		vTaskDelay(100);		
		/**************************固定字符展示与更新*************************************/
		Char_Draw(&GYRO,"084",UI_Graph_ADD,8 ,UI_Color_Yellow,24,4,4,60,890,&gyro_arr[0]);//"GYRO"字符
		Char_ReFresh(GYRO);
		vTaskDelay(100);
		Char_Draw(&CAP,"085",UI_Graph_ADD,8 ,UI_Color_Yellow,24,3,4,60,830,&cap_arr[0]);//"CAP"字符
		Char_ReFresh(CAP);
		vTaskDelay(100);
		Char_Draw(&CH_SHOOT,"087",UI_Graph_ADD,8 ,UI_Color_Yellow,24,5,5,60,770,&shoot_arr[0]);//"SHOOT"字符
		Char_ReFresh(CH_SHOOT);
		vTaskDelay(100);
		Char_Draw(&CH_AUTO,"088",UI_Graph_ADD,8 ,UI_Color_Yellow,24,4,4,60,710,&auto_arr[0]);//"AUTO"字符
		Char_ReFresh(CH_AUTO);
		vTaskDelay(100);
		Char_Draw(&CH_COLLEGE,"086",UI_Graph_ADD,8 ,UI_Color_Yellow,24,5,4,910,880,&college_arr[0]);//"PRIME"字符
		Char_ReFresh(CH_COLLEGE);
		vTaskDelay(100);
		


		/******************************初始化部分变量***********************************/
		uint8_t cap_vol=0;
		uint8_t cap_flag=0;//1,2,3,4代表电量1/4，2/4，3/4，4/4
//    //获取姿态角指针
//    const volatile fp32 *angle;
//    angle = get_INS_angle_point();
//    //获取云台控制结构体指针
//    local_gimbal_control = get_gimbal_control_point();
//    //获取底盘控制结构体指针
//    local_chassis_move = get_chassis_control_point();
//    //初始化卡尔曼滤波结构体
//    Kalman_Filter_Init(&Power_KalmanInfo_Structure);
    while (1)
    {
					/**************************GYRO小陀螺更新部分***********************************/
					if(roation_state==1){
						Char_Draw(&GYRO,"084",UI_Graph_Change,8 ,UI_Color_Purplish_red,24,4,4,60,890,&gyro_arr[0]);//"GYRO"字符
						Char_ReFresh(GYRO);
					}
					else {
						Char_Draw(&GYRO,"084",UI_Graph_Change,8 ,UI_Color_Yellow,24,4,4,60,890,&gyro_arr[0]);//"GYRO"字符
						Char_ReFresh(GYRO);
					}
					vTaskDelay(35);
					/**************************CAP电量更新部分************************************/
//					cap_vol=cap_electric_quantity;
//					//printf("CAP: %d",cap_electric_quantity);
//					if(cap_electric_quantity>0&&cap_electric_quantity<25) {//25%电量
//						cap_flag=1;
//						Circle_Draw(&G8,"098",UI_Graph_Change,9,UI_Color_Purplish_red,15,230,820,15);//"CAP"右侧圆圈1
//						Circle_Draw(&G1,"091",UI_Graph_Change,9,UI_Color_Orange,15,280,820,15);//"CAP"右侧圆圈2
//						UI_ReFresh(2,G1,G8);
//						vTaskDelay(35);
//						Circle_Draw(&G2,"092",UI_Graph_Change,9,UI_Color_Orange,15,330,820,15);//"CAP"右侧圆圈3
//						Circle_Draw(&G3,"093",UI_Graph_Change,9,UI_Color_Orange,15,380,820,15);//"CAP"右侧圆圈4
//						UI_ReFresh(2,G2,G3);
//					}
//					else if(cap_electric_quantity>=25&&cap_electric_quantity<50){
//						cap_flag=2;
//						Circle_Draw(&G8,"098",UI_Graph_Change,9,UI_Color_Purplish_red,15,230,820,15);//"CAP"右侧圆圈1
//						Circle_Draw(&G1,"091",UI_Graph_Change,9,UI_Color_Purplish_red,15,280,820,15);//"CAP"右侧圆圈2
//						UI_ReFresh(2,G1,G8);
//						vTaskDelay(35);
//						Circle_Draw(&G2,"092",UI_Graph_Change,9,UI_Color_Orange,15,330,820,15);//"CAP"右侧圆圈3
//						Circle_Draw(&G3,"093",UI_Graph_Change,9,UI_Color_Orange,15,380,820,15);//"CAP"右侧圆圈4
//						UI_ReFresh(2,G2,G3);
//					} 
//					else if(cap_electric_quantity>=50&&cap_electric_quantity<75){
//						cap_flag=3;
//						Circle_Draw(&G8,"098",UI_Graph_Change,9,UI_Color_Purplish_red,15,230,820,15);//"CAP"右侧圆圈1
//						Circle_Draw(&G1,"091",UI_Graph_Change,9,UI_Color_Purplish_red,15,280,820,15);//"CAP"右侧圆圈2
//						UI_ReFresh(2,G1,G8);
//						vTaskDelay(35);
//						Circle_Draw(&G2,"092",UI_Graph_Change,9,UI_Color_Purplish_red,15,330,820,15);//"CAP"右侧圆圈3
//						Circle_Draw(&G3,"093",UI_Graph_Change,9,UI_Color_Orange,15,380,820,15);//"CAP"右侧圆圈4
//						UI_ReFresh(2,G2,G3);
//					} 
//					else if(cap_electric_quantity>=75&&cap_electric_quantity<=100){
//						cap_flag=4;
//						Circle_Draw(&G8,"098",UI_Graph_Change,9,UI_Color_Purplish_red,15,230,820,15);//"CAP"右侧圆圈1
//						Circle_Draw(&G1,"091",UI_Graph_Change,9,UI_Color_Purplish_red,15,280,820,15);//"CAP"右侧圆圈2
//						UI_ReFresh(2,G1,G8);
//						vTaskDelay(35);
//						Circle_Draw(&G2,"092",UI_Graph_Change,9,UI_Color_Purplish_red,15,330,820,15);//"CAP"右侧圆圈3
//						Circle_Draw(&G3,"093",UI_Graph_Change,9,UI_Color_Purplish_red,15,380,820,15);//"CAP"右侧圆圈4
//						UI_ReFresh(2,G2,G3);
//					} 
//					else cap_flag=0;
//					UI_ReFresh(1,G8);
//					vTaskDelay(35);
					/**************************SHOOT状态更新部分************************************/
					if(shoot_vel_state==1){
							Char_Draw(&CH_SHOOT,"087",UI_Graph_Change,8 ,UI_Color_Purplish_red,24,5,5,60,770,&shoot_arr[0]);//"SHOOT"字符
							Char_ReFresh(CH_SHOOT);
					}
					else {
							Char_Draw(&CH_SHOOT,"087",UI_Graph_Change,8 ,UI_Color_Yellow,24,5,5,60,770,&shoot_arr[0]);//"SHOOT"字符
							Char_ReFresh(CH_SHOOT);
					}
					vTaskDelay(35);
					/**************************AUTO状态更新部分************************************/
					if(auto_state==1){
							Char_Draw(&CH_AUTO,"088",UI_Graph_Change,8 ,UI_Color_Purplish_red,24,4,4,60,710,&auto_arr[0]);//"AUTO"字符
							Char_ReFresh(CH_AUTO);					
					}
					else {
							Char_Draw(&CH_AUTO,"088",UI_Graph_Change,8 ,UI_Color_Yellow,24,4,4,60,710,&auto_arr[0]);//"AUTO"字符
							Char_ReFresh(CH_AUTO);					
					}
					vTaskDelay(35);

//					//频率控制10hz,上限30hz
//          vTaskDelay(100);
					
//					//左右摩擦轮发射UI,初始化黄色，出错紫红色，正常绿色
//				if(shoot_control.shoot_mode==SHOOT_READY||shoot_control.shoot_mode==SHOOT_BULLET||shoot_control.shoot_mode==SHOOT_CONTINUE_BULLET){
//					shoot_flag =1;
//				}
//				else {shoot_flag=0;}
//				if(shoot_flag)
//					Circle_Draw(&G7,"007",UI_Graph_Change,9,UI_Color_Green,15,230,770,15);
//				else Circle_Draw(&G7,"007",UI_Graph_Change,9,UI_Color_Purplish_red,15,230,770,15);
//				//装甲板旋转移动，正常绿色，受击紫红色闪烁，两s反应时间
//				if(heat_flag1==1||heat_flag2==1)
//					Circle_Draw(&G8,"081",UI_Graph_Change,8,UI_Color_Pink,10,960+(int)340*sin((angle)*2*PI/360.0),540+(int)340*cos((angle)*2*PI/360.0),50);
//				else Circle_Draw(&G8,"081",UI_Graph_Change,8,UI_Color_Green,3,960+(int)340*sin((angle)*2*PI/360.0),540+(int)340*cos((angle)*2*PI/360.0),50);
//				if(heat_flag1==2||heat_flag2==2) 
//					Circle_Draw(&G9,"082",UI_Graph_Change,8,UI_Color_Pink,10,960+(int)340*sin((angle+90)*2*PI/360.0),540+(int)340*cos((angle+90)*2*PI/360.0),50);
//				else Circle_Draw(&G9,"082",UI_Graph_Change,8,UI_Color_Green,3,960+(int)340*sin((angle+90)*2*PI/360.0),540+(int)340*cos((angle+90)*2*PI/360.0),50);
//				if(heat_flag1==3||heat_flag2==3) 	
//					Circle_Draw(&G10,"083",UI_Graph_Change,8,UI_Color_Pink,10,960+(int)340*sin((angle+180)*2*PI/360.0),540+(int)340*cos((angle+180)*2*PI/360.0),50);
//				else Circle_Draw(&G10,"083",UI_Graph_Change,8,UI_Color_Green,3,960+(int)340*sin((angle+180)*2*PI/360.0),540+(int)340*cos((angle+180)*2*PI/360.0),50);
//				if(heat_flag1==4||heat_flag2==4) 	
//					Circle_Draw(&G11,"084",UI_Graph_Change,8,UI_Color_Pink,10,960+(int)340*sin((angle+270)*2*PI/360.0),540+(int)340*cos((angle+270)*2*PI/360.0),50);
//				else Circle_Draw(&G11,"084",UI_Graph_Change,8,UI_Color_Green,3,960+(int)340*sin((angle+270)*2*PI/360.0),540+(int)340*cos((angle+270)*2*PI/360.0),50);
//				UI_ReFresh(5, G7, G8, G9, G10, G11);
//				Char_Draw(&CH_FLRB, "077", UI_Graph_Change, 7, UI_Color_Yellow, 24, 1, 4, 960 + (int)340 * sin((angle) * 2 * PI / 360.0), 540 + (int)340 * cos((angle) * 2 * PI / 360.0), &flrb_arr[0]);
//				Char_ReFresh(CH_FLRB);
//				Char_Draw(&CH_FLRB, "076", UI_Graph_Change, 7, UI_Color_Yellow, 24, 1, 4, 960 + (int)340 * sin((angle + 90) * 2 * PI / 360.0), 540 + (int)340 * cos((angle + 90) * 2 * PI / 360.0), &flrb_arr[1]);
//				Char_ReFresh(CH_FLRB);
//				Char_Draw(&CH_FLRB, "075", UI_Graph_Change, 7, UI_Color_Yellow, 24, 1, 4, 960 + (int)340 * sin((angle + 180) * 2 * PI / 360.0), 540 + (int)340 * cos((angle + 180) * 2 * PI / 360.0), &flrb_arr[2]);
//				Char_ReFresh(CH_FLRB);
//				Char_Draw(&CH_FLRB, "074", UI_Graph_Change, 7, UI_Color_Yellow, 24, 1, 4, 960 + (int)340 * sin((angle + 270) * 2 * PI / 360.0), 540 + (int)340 * cos((angle + 270) * 2 * PI / 360.0), &flrb_arr[3]);
//				Char_ReFresh(CH_FLRB);
				/**************************自定义UI界面——END************************************/

//        Tcount++;
//        if(Tcount >= 50)
//        {
//            led_blue_toggle();
//            Tcount = 0;
//        }
//        //姿态角 将rad 变成 度，除这里的姿态角的单位为度，其他地方的姿态角，单位均为弧度
//        // angle_degree[0] = (*(angle + INS_YAW_ADDRESS_OFFSET)) * 57.3f;
//        // angle_degree[1] = (*(angle + INS_PITCH_ADDRESS_OFFSET)) * 57.3f;
//        // angle_degree[2] = (*(angle + INS_ROLL_ADDRESS_OFFSET)) * 57.3f;

//        angle_degree[0] = (*(angle + INS_YAW_ADDRESS_OFFSET));
//        angle_degree[1] = (*(angle + INS_PITCH_ADDRESS_OFFSET));
//        angle_degree[2] = (*(angle + INS_ROLL_ADDRESS_OFFSET));

        
        // printf("%.2f,%.2f,%.2f\r\n",local_gimbal_control->gimbal_pitch_motor.absolute_angle,\
        //                                 local_gimbal_control->gimbal_pitch_motor.absolute_angle_set,\
        //                                 local_gimbal_control->gimbal_pitch_motor.given_current);

        // printf("%.2f,%.2f\r\n",local_gimbal_control->gimbal_yaw_motor.absolute_angle,\
        //                             local_gimbal_control->gimbal_yaw_motor.absolute_angle_set);

        // printf("%.2f,%.2f,%d\r\n",local_gimbal_control->gimbal_yaw_motor.motor_gyro,\
        //                         local_gimbal_control->gimbal_yaw_motor.motor_gyro_set,\
        //                         local_gimbal_control->gimbal_yaw_motor.flag);

        //printf("%.2f\r\n",rad_format( local_gimbal_control->gimbal_yaw_motor.gimbal_motor_measure->pos) );
        //printf("%d\r\n",local_gimbal_control->gimbal_yaw_motor.gimbal_motor_mode);

        //printf("%.2f\r\n",local_chassis_move->chassis_relative_angle);

        //printf("%.2f,%.2f,%.2f\r\n",local_chassis_move->vx_set,local_chassis_move->vy_set,local_chassis_move->wz_set);
        // printf("%.2f,%.2f,%.2f\r\n",local_chassis_move->chassis_relative_angle_set,\
        //     local_chassis_move->chassis_yaw_motor->relative_angle,\
        //     local_chassis_move->wz_set);

        //printf("%d\r\n",local_chassis_move->chassis_mode);

        // printf("%.2f,%.2f,%.2f\r\n",local_gimbal_control->gimbal_yaw_motor.max_relative_angle,
        //     local_gimbal_control->gimbal_yaw_motor.max_relative_angle,
        //     local_gimbal_control->gimbal_yaw_motor.relative_angle);
        //printf("%.2f")
        //printf("%.2f,%.2f\r\n",local_gimbal_control->gimbal_pitch_motor.inti_pitch_angle,local_gimbal_control->gimbal_pitch_motor.absolute_angle);
        
        // printf("%.2f,%.2f,%.2f\r\n",local_gimbal_control->gimbal_yaw_motor.relative_angle,\
        //         local_gimbal_control->gimbal_yaw_motor.relative_angle_set,\
        //         local_gimbal_control->gimbal_yaw_motor.motor_gyro);
        	

#if INCLUDE_uxTaskGetStackHighWaterMark
        UserTaskStack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

//计算底盘功率
fp32 Power_Calc(void)
{
    fp32 battery_voltage = get_battery_voltage() + VOLTAGE_DROP;
    fp32 power = 0;
    for(int i=0;i<4;i++)
    {
        fp32 temp_current = (fp32)local_chassis_move->motor_chassis[i].chassis_motor_measure->given_current / 1000.0f / 2.75f;
        if(temp_current < 0.0f)
            temp_current = -temp_current;
        power += battery_voltage * temp_current / 1.414f;
    }
    fp32 new_power = Kalman_Filter_Fun(&Power_KalmanInfo_Structure,power);
    return new_power;
}
