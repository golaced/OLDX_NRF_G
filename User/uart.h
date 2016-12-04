#ifndef __USART_H_
#define __USART_H_
#include "stm32f10x.h"


void usart1_config(void);
uint8_t UART1_Put_Char(unsigned char DataToSend);
void GOL_LINK_TASK(void);

//nav_board
struct _IMU{
		float pitch;
		float roll;
		float yaw;
	  int gro_x;
		int gro_y;
		int gro_z;
		int acc_x;
		int acc_y;
		int acc_z;
		int q0;
		int q1;
		int q2;
		int q3;
		long J;
		long W;
		int H;
	      };

struct _SENSOR{
	int accx;
	int accy;
	int accz;
	int grox;
	int groy;
	int groz;
	int hmx;
	int hmy;
	int hmz;
	int bmp;
	int temp;
	int sonar;
	      };

struct _FUSHION{
	int alt;
	int spd;
	int alt_acc_bmp;
	int alt_acc_sonar;
	int spd_acc_bmp;
	int spd_acc_sonar;
	      };
struct _IMU_NAV1{
	int alt;
	int spd_w;
	int spd_e;
	int spd_h;
	int spd_x;
	int spd_y;
	long DJ,DW;
	long GJ,GW;
	      };				
struct _NAV{
				struct _IMU imu;    
				struct _SENSOR sensor;  
				struct _FUSHION fushion; 
				struct _IMU_NAV1 imu_nav;
	      u8 fame;
            };

extern struct _NAV nav;
						
struct _SET{
float pitch;
float roll;
float yaw;
int alt;
int alt_bmp;
int alt_fushion;
int spd_x;
int spd_y;
int pos[2];
int pos_t[2];
int spd[2];
int spd_f[2];	
float nav[2];
float spd_alt;
int thr;	

		};

struct _STATE{
u8 alt_data ;
u8 gps ;
u8 flow;
u8 imu_nav;
u8 pid_fuzzy ;
		};	

struct _PID{
u16 pp_o;
u16 pi_o;
u16 pd_o;
u16 pp_i;
u16 pi_i;
u16 pd_i;

u16 rp_o;
u16 ri_o;
u16 rd_o;
u16 rp_i;
u16 ri_i;
u16 rd_i;

u16 yp_o;
u16 yi_o;
u16 yd_o;
u16 yp_i;
u16 yi_i;
u16 yd_i;

u16 hp_o;
u16 hi_o;
u16 hd_o;
u16 hp_i;
u16 hi_i;
u16 hd_i;

		};	

struct _PIDOUT{
int pitch;
int roll;
int yaw;
int alt;
int spd_x;
int spd_y;

		};	
struct _POS_GPS_NAV1 {
long J;
long W;
long X_O;
long Y_O;
long X_UKF;
long Y_UKF;
u8 gps_mode;
u8 star_num;
};

extern u8 Not_sent;
struct _FLY{
				struct _IMU imu;    
				struct _SENSOR sensor;  
				struct _SET set; 
				struct _SET flow; 	
				struct _SET now;  
	      struct _STATE state;
	      struct _PID pid;
	      struct _PIDOUT pid_out;
				struct _POS_GPS_NAV1 gps;
	      int slam_sonar[5];
	      u8 fame;
            };

extern struct _FLY fly_controller;
			
void UART2_ReportMotion(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,
					int16_t hx,int16_t hy,int16_t hz);
void UART2_ReportIMU(int16_t yaw,int16_t pitch,int16_t roll
,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec);						
#endif
