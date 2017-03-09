/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * ???  :RC.c
 * ??    :???????         
 * ????:Air Nano?????
 * ???  :ST3.5.0
 * ??    :Air Nano Team 
 * ??    :http://byd2.taobao.com
**********************************************************************************/
#include "head.h"
#include "led.h"

#define RX_DR			6		//????
#define TX_DS			5
#define MAX_RT		4
int DEBUG[35];
int BLE_DEBUG[16];
float Pitch,Roll,Yaw;
vs16 QH,ZY,XZ;
u8 is_lock=1;
u8 EN_FIX_GPSF=0;
u8 EN_FIX_LOCKWF=0;
u8 EN_CONTROL_IMUF=0;
u8 EN_FIX_INSF=0;
u8 EN_FIX_HIGHF=0;
u8 tx_lock=1;
u8 EN_FIX_GPS=0;
u8 EN_FIX_LOCKW=0;
u8 EN_CONTROL_IMU=0;
u8 EN_FIX_INS=0;
u8 EN_FIX_HIGH=0;
u8 EN_TX_GX=1;
u8 EN_TX_AX=1;
u8 EN_TX_HM=1;
u8 EN_TX_YRP=1;
u8 EN_TX_GPS=1;
u8 EN_TX_HIGH=1;
u8 up_load_set=0;
u8 up_load_pid=0;
u8 key_rc[6]={1,1,1,1,1,1};
u16 Yaw_sb_rc=0;

u8 cnt_rst=0,delta_pitch=0,delta_roll=0,delta_yew=0;

 PID_STA HPIDt,SPIDt;	
 u16 ax,ay,az,gx,gy,gz,hx,hy,hz,YM,PWM1,PWM2,PWM3,PWM4,fix_pit,fix_rol;

u16 bat_fly=840,high_f;
float ypr[3];
double GPS_W,GPS_J;
u8 data_rate,fly_mode;
int rc_rate_cnt[10];
PID_STA HPID,SPID,FIX_PID,NAV_PID;
struct _PID_SET pid;
RC_GETDATA Rc_Get;
struct _plane plane;
struct _slam slam;
struct _IMU_NAV imu_nav;
struct _MODE mode;
int qr_pos[2],tar_pos[2],drone_pos[3];
int yaw_gimbal;
u8 state_drone,need_avoid,tar_check;

void NRF_DataAnl(void)
{ 
u8 temp_key[7];
u8 temp;
	u8 i=0,j,sum = 0;
	for( i=0;i<31;i++)
		sum += NRF24L01_RXDATA[i];
	if(!(sum==NRF24L01_RXDATA[31]))		
		return;		//??sum
	if(!(NRF24L01_RXDATA[0]==0x88))		
		return;		//????
	
	if(NRF24L01_RXDATA[1]==0x1)								//?D??1|?ü×?,=0x8a,?aò￡??êy?Y
	{ //TIM_Cmd(TIM2,DISABLE);
		//cnt_timer2_reg=cnt_timer2;
		bat_fly=((vs16)(NRF24L01_RXDATA[3]<<8)|NRF24L01_RXDATA[4]);
		GPS_W=((NRF24L01_RXDATA[5]<<24)|(NRF24L01_RXDATA[6]<<16)|(NRF24L01_RXDATA[7]<<8)|NRF24L01_RXDATA[8])/10000000.;
		GPS_J=((NRF24L01_RXDATA[9]<<24)|(NRF24L01_RXDATA[10]<<16)|(NRF24L01_RXDATA[11]<<8)|NRF24L01_RXDATA[12])/10000000.;
		ypr[0]=((NRF24L01_RXDATA[13]<<8)|NRF24L01_RXDATA[14])/100.;
		ypr[1]=((NRF24L01_RXDATA[15]<<8)|NRF24L01_RXDATA[16])/100.;
		ypr[2]=((NRF24L01_RXDATA[17]<<8)|NRF24L01_RXDATA[18])/100.;		
		for(j=0;j<3;j++)
		  if(ypr[j]>360)
				ypr[j]-=655.35;
	  YM=(float)((NRF24L01_RXDATA[19]<<8)|NRF24L01_RXDATA[20])*50./62;
	  data_rate=(NRF24L01_RXDATA[21]<<8)|NRF24L01_RXDATA[22];
		//mode.mode_fly=fly_mode=NRF24L01_RXDATA[23];
		high_f= (float)((NRF24L01_RXDATA[24]<<8)|NRF24L01_RXDATA[25]);
		//gps_mode=(NRF24L01_RXDATA[21]);
		//fly_mode=(NRF24L01_RXDATA[22]);
		//gps_no=(NRF24L01_RXDATA[23]);
		//is_lock=(NRF24L01_RXDATA[24]);
		EN_FIX_GPSF=(NRF24L01_RXDATA[25]);
		EN_FIX_LOCKWF=(NRF24L01_RXDATA[26]);
		EN_CONTROL_IMUF=(NRF24L01_RXDATA[27]);
		EN_FIX_INSF=(NRF24L01_RXDATA[28]);
		EN_FIX_HIGHF=(NRF24L01_RXDATA[29]);
			 rc_rate_cnt[0]++;
	}
	else 		if(NRF24L01_RXDATA[1]==0x2)	//接收PID1
		{ rc_rate_cnt[1]++;
			SPIDt.OP = (float)((vs16)(NRF24L01_RXDATA[4]<<8)|NRF24L01_RXDATA[5]);
			SPIDt.OI = (float)((vs16)(NRF24L01_RXDATA[6]<<8)|NRF24L01_RXDATA[7]);
			SPIDt.OD = (float)((vs16)(NRF24L01_RXDATA[8]<<8)|NRF24L01_RXDATA[9]);
			SPIDt.IP = (float)((vs16)(NRF24L01_RXDATA[10]<<8)|NRF24L01_RXDATA[11]);
			SPIDt.II = (float)((vs16)(NRF24L01_RXDATA[12]<<8)|NRF24L01_RXDATA[13]);
			SPIDt.ID = (float)((vs16)(NRF24L01_RXDATA[14]<<8)|NRF24L01_RXDATA[15]);
			SPIDt.YP = (float)((vs16)(NRF24L01_RXDATA[16]<<8)|NRF24L01_RXDATA[17]);
			SPIDt.YI = (float)((vs16)(NRF24L01_RXDATA[18]<<8)|NRF24L01_RXDATA[19]);
			SPIDt.YD = (float)((vs16)(NRF24L01_RXDATA[20]<<8)|NRF24L01_RXDATA[21]);
		//	EE_SAVE_PID();
		}
	else 		if(NRF24L01_RXDATA[1]==0x3)	//接收PID2
		{ rc_rate_cnt[2]++;
			HPIDt.OP = (float)((vs16)(NRF24L01_RXDATA[4]<<8)|NRF24L01_RXDATA[5]);
			HPIDt.OI = (float)((vs16)(NRF24L01_RXDATA[6]<<8)|NRF24L01_RXDATA[7]);
			HPIDt.OD = (float)((vs16)(NRF24L01_RXDATA[8]<<8)|NRF24L01_RXDATA[9]);
			fix_pit = ((vs16)(NRF24L01_RXDATA[10]<<8)|NRF24L01_RXDATA[11]);
		  fix_rol =((vs16)(NRF24L01_RXDATA[12]<<8)|NRF24L01_RXDATA[13]);
			
		//	EE_SAVE_PID();
		}
	else 		if(NRF24L01_RXDATA[1]==0x4)	//接收sensor
	{ rc_rate_cnt[3]++;
			ax = ((vs16)(NRF24L01_RXDATA[3]<<8)|NRF24L01_RXDATA[4]);
			ay = ((vs16)(NRF24L01_RXDATA[5]<<8)|NRF24L01_RXDATA[6]);
			az = ((vs16)(NRF24L01_RXDATA[7]<<8)|NRF24L01_RXDATA[8]);
			gx = ((vs16)(NRF24L01_RXDATA[9]<<8)|NRF24L01_RXDATA[10]);
			gy = ((vs16)(NRF24L01_RXDATA[11]<<8)|NRF24L01_RXDATA[12]);
			gz = ((vs16)(NRF24L01_RXDATA[13]<<8)|NRF24L01_RXDATA[14]);
			hx = ((vs16)(NRF24L01_RXDATA[15]<<8)|NRF24L01_RXDATA[16]);
			hy = ((vs16)(NRF24L01_RXDATA[17]<<8)|NRF24L01_RXDATA[18]);
			hz = ((vs16)(NRF24L01_RXDATA[19]<<8)|NRF24L01_RXDATA[20]);
		
		  ;//YM = ((vs16)(NRF24L01_RXDATA[21]<<8)|NRF24L01_RXDATA[22]);
			PWM1 = ((vs16)(NRF24L01_RXDATA[23]<<8)|NRF24L01_RXDATA[24]);
			PWM2 = ((vs16)(NRF24L01_RXDATA[25]<<8)|NRF24L01_RXDATA[26]);
			PWM3 = ((vs16)(NRF24L01_RXDATA[27]<<8)|NRF24L01_RXDATA[28]);
			PWM4 = ((vs16)(NRF24L01_RXDATA[29]<<8)|NRF24L01_RXDATA[30]);
		  
	//	EE_SAVE_PID();
	}
 else 		if(NRF24L01_RXDATA[1]==0x5)	//GPS
	{  rc_rate_cnt[4]++;
			imu_nav.gps.J = 			(NRF24L01_RXDATA[4]<<24)|(NRF24L01_RXDATA[5]<<16)|(NRF24L01_RXDATA[6]<<8)|NRF24L01_RXDATA[7];
			imu_nav.gps.W = 			(NRF24L01_RXDATA[8]<<24)|(NRF24L01_RXDATA[9]<<16)|(NRF24L01_RXDATA[10]<<8)|NRF24L01_RXDATA[11];
			imu_nav.gps.gps_mode = NRF24L01_RXDATA[12];
			imu_nav.gps.star_num = NRF24L01_RXDATA[13];
			imu_nav.gps.X_O = 		(NRF24L01_RXDATA[14]<<24)|(NRF24L01_RXDATA[15]<<16)|(NRF24L01_RXDATA[16]<<8)|NRF24L01_RXDATA[17];
			imu_nav.gps.Y_O = 		(NRF24L01_RXDATA[18]<<24)|(NRF24L01_RXDATA[19]<<16)|(NRF24L01_RXDATA[20]<<8)|NRF24L01_RXDATA[21];
			imu_nav.gps.X_UKF =	  (NRF24L01_RXDATA[22]<<24)|(NRF24L01_RXDATA[23]<<16)|(NRF24L01_RXDATA[24]<<8)|NRF24L01_RXDATA[25];
			imu_nav.gps.Y_UKF = 	(NRF24L01_RXDATA[26]<<24)|(NRF24L01_RXDATA[27]<<16)|(NRF24L01_RXDATA[28]<<8)|NRF24L01_RXDATA[29];
	}
 else 		if(NRF24L01_RXDATA[1]==0x30)	//Debug1
	{  
		
			DEBUG[1] = (float)((vs16)(NRF24L01_RXDATA[4]<<8)|NRF24L01_RXDATA[5]);
			DEBUG[2] = (float)((vs16)(NRF24L01_RXDATA[6]<<8)|NRF24L01_RXDATA[7]);
			DEBUG[3] = (float)((vs16)(NRF24L01_RXDATA[8]<<8)|NRF24L01_RXDATA[9]);
			DEBUG[4] = (float)((vs16)(NRF24L01_RXDATA[10]<<8)|NRF24L01_RXDATA[11]);
			DEBUG[5] = (float)((vs16)(NRF24L01_RXDATA[12]<<8)|NRF24L01_RXDATA[13]);
			DEBUG[6] = (float)((vs16)(NRF24L01_RXDATA[14]<<8)|NRF24L01_RXDATA[15]);
			DEBUG[7] = (float)((vs16)(NRF24L01_RXDATA[16]<<8)|NRF24L01_RXDATA[17]);
			DEBUG[8] = (float)((vs16)(NRF24L01_RXDATA[18]<<8)|NRF24L01_RXDATA[19]);
			DEBUG[9] = (float)((vs16)(NRF24L01_RXDATA[20]<<8)|NRF24L01_RXDATA[21]);
		  DEBUG[10]= (float)((vs16)(NRF24L01_RXDATA[22]<<8)|NRF24L01_RXDATA[23]);
			DEBUG[11]= (float)((vs16)(NRF24L01_RXDATA[24]<<8)|NRF24L01_RXDATA[25]);
			DEBUG[12]= (float)((vs16)(NRF24L01_RXDATA[28]<<8)|NRF24L01_RXDATA[27]);
		  DEBUG[13]= (float)((vs16)(NRF24L01_RXDATA[30]<<8)|NRF24L01_RXDATA[29]);
	}	
 else 		if(NRF24L01_RXDATA[1]==0x31)	//Debug1
	{  
		
			DEBUG[14] = (float)((vs16)(NRF24L01_RXDATA[4]<<8)|NRF24L01_RXDATA[5]);
			DEBUG[15] = (float)((vs16)(NRF24L01_RXDATA[6]<<8)|NRF24L01_RXDATA[7]);
			DEBUG[16] = (float)((vs16)(NRF24L01_RXDATA[8]<<8)|NRF24L01_RXDATA[9]);
			DEBUG[17] = (float)((vs16)(NRF24L01_RXDATA[10]<<8)|NRF24L01_RXDATA[11]);
			DEBUG[18] = (float)((vs16)(NRF24L01_RXDATA[12]<<8)|NRF24L01_RXDATA[13]);
			DEBUG[19] = (float)((vs16)(NRF24L01_RXDATA[14]<<8)|NRF24L01_RXDATA[15]);
			DEBUG[20] = (float)((vs16)(NRF24L01_RXDATA[16]<<8)|NRF24L01_RXDATA[17]);
			DEBUG[21] = (float)((vs16)(NRF24L01_RXDATA[18]<<8)|NRF24L01_RXDATA[19]);
			DEBUG[22] = (float)((vs16)(NRF24L01_RXDATA[20]<<8)|NRF24L01_RXDATA[21]);
		  DEBUG[23]= (float)((vs16)(NRF24L01_RXDATA[22]<<8)|NRF24L01_RXDATA[23]);
			DEBUG[24]= (float)((vs16)(NRF24L01_RXDATA[24]<<8)|NRF24L01_RXDATA[25]);
			DEBUG[25]= (float)((vs16)(NRF24L01_RXDATA[28]<<8)|NRF24L01_RXDATA[27]);
		  DEBUG[26]= (float)((vs16)(NRF24L01_RXDATA[30]<<8)|NRF24L01_RXDATA[29]);
	}	
	else 		if(NRF24L01_RXDATA[1]==0x08)	//Debug1
	{  
		
			BLE_DEBUG[0] = ((vs16)(NRF24L01_RXDATA[3]<<8)|NRF24L01_RXDATA[4]);
			BLE_DEBUG[1] = ((vs16)(NRF24L01_RXDATA[5]<<8)|NRF24L01_RXDATA[6]);
			BLE_DEBUG[2] = ((vs16)(NRF24L01_RXDATA[7]<<8)|NRF24L01_RXDATA[8]);
			BLE_DEBUG[3] = ((vs16)(NRF24L01_RXDATA[9]<<8)|NRF24L01_RXDATA[10]);
			BLE_DEBUG[4] = ((vs16)(NRF24L01_RXDATA[11]<<8)|NRF24L01_RXDATA[12]);
			BLE_DEBUG[5] = ((vs16)(NRF24L01_RXDATA[13]<<8)|NRF24L01_RXDATA[14]);
			BLE_DEBUG[6] = ((vs16)(NRF24L01_RXDATA[15]<<8)|NRF24L01_RXDATA[16]);
			BLE_DEBUG[7] = ((vs16)(NRF24L01_RXDATA[17]<<8)|NRF24L01_RXDATA[18]);
			BLE_DEBUG[8] = ((vs16)(NRF24L01_RXDATA[19]<<8)|NRF24L01_RXDATA[20]);
		  BLE_DEBUG[9]=  ((vs16)(NRF24L01_RXDATA[21]<<8)|NRF24L01_RXDATA[22]);
			BLE_DEBUG[10]= ((vs16)(NRF24L01_RXDATA[23]<<8)|NRF24L01_RXDATA[24]);
			BLE_DEBUG[11]= ((vs16)(NRF24L01_RXDATA[25]<<8)|NRF24L01_RXDATA[26]);
		  BLE_DEBUG[12]= ((vs16)(NRF24L01_RXDATA[27]<<8)|NRF24L01_RXDATA[28]);
	}	
	 else	if(NRF24L01_RXDATA[1]==0x66)	//APP
	{  
		

			state_drone = (float)((vs16)(NRF24L01_RXDATA[3]<<8)|NRF24L01_RXDATA[4]);
			drone_pos[2] = (float)((vs16)(NRF24L01_RXDATA[5]<<8)|NRF24L01_RXDATA[6]);
			tar_pos[0] = (float)((vs16)(NRF24L01_RXDATA[7]<<8)|NRF24L01_RXDATA[8]);
			tar_pos[1] = (float)((vs16)(NRF24L01_RXDATA[9]<<8)|NRF24L01_RXDATA[10]);
			drone_pos[0] = (float)((vs16)(NRF24L01_RXDATA[11]<<8)|NRF24L01_RXDATA[12]);
			drone_pos[1] = (float)((vs16)(NRF24L01_RXDATA[13]<<8)|NRF24L01_RXDATA[14]);
			qr_pos[0] = (float)((vs16)(NRF24L01_RXDATA[15]<<8)|NRF24L01_RXDATA[16]);
			qr_pos[1] = (float)((vs16)(NRF24L01_RXDATA[17]<<8)|NRF24L01_RXDATA[18]);
			yaw_gimbal = (float)((vs16)(NRF24L01_RXDATA[19]<<8)|NRF24L01_RXDATA[20]);
		  need_avoid = NRF24L01_RXDATA[21];
		  tar_check = NRF24L01_RXDATA[22];
		 // DEBUG[23]= (float)((vs16)(NRF24L01_RXDATA[22]<<8)|NRF24L01_RXDATA[23]);
			//DEBUG[24]= (float)((vs16)(NRF24L01_RXDATA[24]<<8)|NRF24L01_RXDATA[25]);
			//DEBUG[25]= (float)((vs16)(NRF24L01_RXDATA[28]<<8)|NRF24L01_RXDATA[27]);
		  //DEBUG[26]= (float)((vs16)(NRF24L01_RXDATA[30]<<8)|NRF24L01_RXDATA[29]);
		
		
	}
}


int cnt_timer2_r=0;
u8 cnt_led_rx=0;
void Nrf_Check_Event(void)
{ 
	u8 rx_len =0;
	static u16 cnt_loss_rc=0;
	u8 sta;

	sta= NRF_Read_Reg(NRF_READ_REG + NRFRegSTATUS);		//??2401?????
	////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////
	if(sta & (1<<RX_DR))	//??ing 
	{ 
		cnt_loss_rc=0;
			
				
				
				rx_len= NRF_Read_Reg(R_RX_PL_WID);
				
				if(rx_len<33)	//??????33?????,???????
				{
					NRF_Read_Buf(RD_RX_PLOAD,NRF24L01_RXDATA,RX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer
					NRF_DataAnl();	//??2401??????
				 static u8 ledtx;
				if(ledtx)ledtx=0;
				else ledtx=1;
				LED_STATE(RX_LED,ledtx);
				}
				else 
				{ 
					NRF_Write_Reg(FLUSH_RX,0xff);//?????
				}
				if(cnt_led_rx<2)
				cnt_led_rx++;
				else 
				{
				cnt_led_rx=0;
				}
	}
	else//---------losing_nrf
	 {	
		 if(cnt_loss_rc++>200 )//0.5ms
		 {	NRF_Write_Reg(FLUSH_RX,0xff);//?????
			
	  	//RX_CH[PIT]=RX_CH[ROL]=RX_CH[YAW]=1500;

			data_rate=0;
		 }
  }
	////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////
	if(sta & (1<<TX_DS))	//????,?????
	{
	
	}
	////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////
	
	if(sta & (1<<MAX_RT))//??,????
	{
		if(sta & 0x01)	//TX FIFO FULL
		{
			NRF_Write_Reg(FLUSH_TX,0xff);
		}
	}
	////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////
	NRF_Write_Reg(NRF_WRITE_REG + NRFRegSTATUS, sta);
}


void NRF_Send_ARMED(void)//????
{
	uint8_t i;
	vs16 _temp;	
	u8 cnt=0;
  
	NRF24L01_TXDATA[0] = 0x8A;	//源程序中为8A	遥控指令
	NRF24L01_TXDATA[1] = 0x8A;	//源程序中为8A	遥控指令
	NRF24L01_TXDATA[2] = 0x1C;
	NRF24L01_TXDATA[3] = BYTE1(Rc_Get.THROTTLE);
	NRF24L01_TXDATA[4] = BYTE0(Rc_Get.THROTTLE);

	NRF24L01_TXDATA[5] = BYTE1(Rc_Get.YAW);
	NRF24L01_TXDATA[6] = BYTE0(Rc_Get.YAW);
	NRF24L01_TXDATA[7] = BYTE1(Rc_Get.ROLL);
	NRF24L01_TXDATA[8] = BYTE0(Rc_Get.ROLL);
	NRF24L01_TXDATA[9] = BYTE1(Rc_Get.PITCH);
	NRF24L01_TXDATA[10] = BYTE0(Rc_Get.PITCH);
	
	NRF24L01_TXDATA[11] = BYTE1(Rc_Get.AUX1);
	NRF24L01_TXDATA[12] = BYTE0(Rc_Get.AUX1);
	NRF24L01_TXDATA[13] = BYTE1(Rc_Get.AUX2);
	NRF24L01_TXDATA[14] = BYTE0(Rc_Get.AUX2);
	NRF24L01_TXDATA[15] = BYTE1(Rc_Get.AUX3);
	NRF24L01_TXDATA[16] = BYTE0(Rc_Get.AUX3);
	NRF24L01_TXDATA[17] = BYTE1(Rc_Get.AUX4);
	NRF24L01_TXDATA[18] = BYTE0(Rc_Get.AUX4);
	NRF24L01_TXDATA[19] = BYTE1(Rc_Get.AUX5);
	NRF24L01_TXDATA[20] = BYTE0(Rc_Get.AUX5);
	NRF24L01_TXDATA[21] =(key_sel[3]<<3)|(key_sel[2]<<2)|(key_sel[1]<<1)|key_sel[0];
	NRF24L01_TXDATA[22] =(key[7]<<7)|(key[6]<<6)|(key[5]<<5)|(key[4]<<4)|(key[3]<<3)|(key[2]<<2)|(key[1]<<1)|key[0];
	_temp=(int)(Yaw*100);
	NRF24L01_TXDATA[23] = BYTE1(_temp);
	NRF24L01_TXDATA[24] = BYTE0(_temp);
	_temp=(int)(Pitch*100);
	NRF24L01_TXDATA[25] = BYTE1(_temp);
	NRF24L01_TXDATA[26] = BYTE0(_temp);
	_temp=(int)(Roll*100);
	NRF24L01_TXDATA[27] = BYTE1(_temp);
	NRF24L01_TXDATA[28] = BYTE0(_temp);
  u8 sum = 0;
	for(u8 i=0;i<31;i++)
		sum += NRF24L01_TXDATA[i];
	NRF24L01_TXDATA[31] = sum;
	
	NRF_TxPacket_AP(NRF24L01_TXDATA,32);
}

void NRF_Send_RC_PID1(void)//????
{	vs16 _temp;	u8 i;	u8 sum = 0;

	NRF24L01_TXDATA[0] = 0x8A;	//源程序中为8BSET指令
	NRF24L01_TXDATA[1] = 0x8C;	//源程序中为8BSET指令
	NRF24L01_TXDATA[2] = 0x1C;
	NRF24L01_TXDATA[3]=0xAD;
	
	_temp = SPID.OP * 1;
	NRF24L01_TXDATA[4]=BYTE1(_temp);
	NRF24L01_TXDATA[5]=BYTE0(_temp);
	_temp = SPID.OI * 1;
	NRF24L01_TXDATA[6]=BYTE1(_temp);
	NRF24L01_TXDATA[7]=BYTE0(_temp);
	_temp = SPID.OD * 1;
	NRF24L01_TXDATA[8]=BYTE1(_temp);
	NRF24L01_TXDATA[9]=BYTE0(_temp);
	_temp = SPID.IP * 1;
	NRF24L01_TXDATA[10]=BYTE1(_temp);
	NRF24L01_TXDATA[11]=BYTE0(_temp);
	_temp = SPID.II * 1;
	NRF24L01_TXDATA[12]=BYTE1(_temp);
	NRF24L01_TXDATA[13]=BYTE0(_temp);
	_temp = SPID.ID * 1;
	NRF24L01_TXDATA[14]=BYTE1(_temp);
	NRF24L01_TXDATA[15]=BYTE0(_temp);
	_temp = SPID.YP * 1;
	NRF24L01_TXDATA[16]=BYTE1(_temp);
	NRF24L01_TXDATA[17]=BYTE0(_temp);
	_temp = SPID.YI * 1;
	NRF24L01_TXDATA[18]=BYTE1(_temp);
	NRF24L01_TXDATA[19]=BYTE0(_temp);
	_temp = SPID.YD * 1;
	NRF24L01_TXDATA[20]=BYTE1(_temp);
	NRF24L01_TXDATA[21]=BYTE0(_temp);
	
	
	
sum = 0;
	for( i=0;i<31;i++)
		sum += NRF24L01_TXDATA[i];
	NRF24L01_TXDATA[31] = sum;
	NRF_TxPacket_AP(NRF24L01_TXDATA,32);
}

void NRF_Send_RC_PID2(void)//????
{	vs16 _temp;	u8 _cnt=0;u8 i;u8 sum = 0;

	NRF24L01_TXDATA[_cnt++] = 0x8A;	//源程序中为8BSET指令
	NRF24L01_TXDATA[_cnt++] = 0x8D;	//源程序中为8BSET指令
	NRF24L01_TXDATA[_cnt++] = 0x1C;
	NRF24L01_TXDATA[_cnt++]=0xAD;
	_temp = 	  HPID.OP;
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
		_temp = 	HPID.OI;
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
		_temp = 	HPID.OD;
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	
	sum = 0;
	
	for( i=0;i<31;i++)
		sum += NRF24L01_TXDATA[i];
	NRF24L01_TXDATA[31] = sum;
		NRF_TxPacket_AP(NRF24L01_TXDATA,32);
}

void NRF_Send_RC_SET(void)//????
{	vs16 _temp;	u8 _cnt=0;u8 i;u8 sum = 0;

NRF24L01_TXDATA[0] = 0x8A;	//源程序中为8BSET指令
	NRF24L01_TXDATA[1] = 0x8B;	//源程序中为8BSET指令
	NRF24L01_TXDATA[2] = 0x1C;
	
	
	
	NRF24L01_TXDATA[3]=tx_lock;
	NRF24L01_TXDATA[4]= EN_FIX_GPS;
	NRF24L01_TXDATA[5]= EN_FIX_LOCKW;
	NRF24L01_TXDATA[6]= EN_CONTROL_IMU;
	NRF24L01_TXDATA[7]= EN_FIX_INS;
	NRF24L01_TXDATA[8]= EN_FIX_HIGH;
	
	NRF24L01_TXDATA[9]= EN_TX_GX;
	NRF24L01_TXDATA[10]= EN_TX_AX;
	NRF24L01_TXDATA[11]= EN_TX_HM;
	NRF24L01_TXDATA[12]= EN_TX_YRP;
	NRF24L01_TXDATA[13]= EN_TX_GPS;
	NRF24L01_TXDATA[14]= EN_TX_HIGH;
	
	sum = 0;
	for( i=0;i<31;i++)
		sum += NRF24L01_TXDATA[i];
	NRF24L01_TXDATA[31] = sum;
		NRF_TxPacket_AP(NRF24L01_TXDATA,32);
}
	
void RC_Send_Task(void)
{
static u16 cnt[4]={0,0,0,0};

if(cnt[0]++>0)
{NRF_Send_ARMED();//????
 cnt[0]=0;}
if(cnt[1]++>1)
{	
 NRF_Send_RC_SET();//????
 cnt[1]=0;
}
if(cnt[2]++>2)
{ NRF_Send_RC_PID1();//????
 cnt[2]=0;}

if(cnt[3]++>2)
{ NRF_Send_RC_PID2();//????
 cnt[3]=0;}

static u8 ledtx;
if(ledtx)ledtx=0;
else ledtx=1;
LED_STATE(TX_LED,ledtx);
}


void CAL_CHECK(void)
{
	/*static u8 state_mpu,state_hml;
static u16 cnt_mpu,cnt_hml;
static u8 check_num_mpu,check_num_hml;
	if(!Mag_CALIBRATED&&!fly_ready)
	switch (state_mpu)
	{
		case 0:
			 if(Rc_Get.YAW-1500>200&&!SEL_KEY[0])
				  state_mpu=1;
			 break;
		case 1:
			 if(Rc_Get.YAW-1500<-200&&!SEL_KEY[0])
				  state_mpu=2;
			 else if(cnt_mpu++>2000)
			 {cnt_mpu=0;state_mpu=0;check_num_mpu=0;}
			 else if(check_num_mpu>6)
			 {  state_mpu=3;
	LEDRGB_COLOR(YELLOW);Delay_ms(100);
	LEDRGB_COLOR(BLACK);Delay_ms(100);
	LEDRGB_COLOR(YELLOW);Delay_ms(100);
	LEDRGB_COLOR(BLACK);Delay_ms(100);
	LEDRGB_COLOR(YELLOW);Delay_ms(100);
	LEDRGB_COLOR(BLACK);Delay_ms(100);
	LEDRGB_COLOR(YELLOW);Delay_ms(500);
				 
				 mpu6050.Gyro_CALIBRATE=1;
			  mpu6050.Acc_CALIBRATE=1;}
			  break;
		case 2:
			if(Rc_Get.YAW-1500>200&&!SEL_KEY[0])
			{  state_mpu=1;check_num_mpu++;}
			 else if(cnt_mpu++>2000)
			 {cnt_mpu=0;state_mpu=0;check_num_mpu=0;}
			  break;
		case 3:	 
			 if(!mpu6050.Gyro_CALIBRATE)
			 {  
			 cnt_mpu=0;state_mpu=0;check_num_mpu=0;}
			break; 
				
	}
	else
	{cnt_mpu=0;state_mpu=0;check_num_mpu=0;}

if( !mpu6050.Gyro_CALIBRATE&&!fly_ready)
switch (state_hml)
	{
		case 0:
			 if(Rc_Get.YAW-1500>200&&SEL_KEY[0])
				  state_hml=1;
			 break;
		case 1:
			 if(Rc_Get.YAW-1500<-200&&SEL_KEY[0])
				  state_hml=2;
			 else if(cnt_hml++>2000)
			 {state_hml=0;cnt_hml=0;check_num_hml=0;}
			 else if(check_num_hml>6)
			 {  state_hml=3;
			LEDRGB_COLOR(BLUE);Delay_ms(100);
			LEDRGB_COLOR(BLACK);Delay_ms(100);
			LEDRGB_COLOR(BLUE);Delay_ms(100);
			LEDRGB_COLOR(BLACK);Delay_ms(100);
			LEDRGB_COLOR(BLUE);Delay_ms(100);
			LEDRGB_COLOR(BLACK);Delay_ms(100);
			LEDRGB_COLOR(BLUE);Delay_ms(500);Mag_CALIBRATED=1;				 
			 }
			  break;
		case 2:
			if(Rc_Get.YAW-1500>200&&SEL_KEY[0])
			{  state_hml=1;check_num_hml++;}
			 else if(cnt_hml++>2000)
			 {state_hml=0;cnt_hml=0;check_num_hml=0;}
			  break;
		case 3:	 
			 if(!Mag_CALIBRATED)
			 {  //flag.calibratingM=0;
			   
			 cnt_hml=0;state_hml=0;check_num_hml=0;}
			break; 
				
	}
	else
	{
	cnt_hml=0;state_hml=0;check_num_hml=0;
	}
*/
}

