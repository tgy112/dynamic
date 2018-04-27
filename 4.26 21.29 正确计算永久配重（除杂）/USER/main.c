/*
本程序为动平衡的算法实现，可以求出影响系数K的值以及永久配置M的值

输入引脚：PB13	周期输入信号
					PA8		32倍频输入信号
					PA1		AD输入通道1
					PA2		AD输入通道2
*/
#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "can.h"
#include "adc.h"
#include "timer.h"
#include "exti.h"
#include "math.h"
											
#define ROW 100
#define COL 100
#define pi 	3.14159
struct polar_coordinate//创建矢量结构体
{
	float size;
	float angle;
};
struct rectangular_coordinate
{
	float x;
	float y;
};
/*****************************************************
***********
*********   数组计算相关函数(以下)
***********
*****************************************************/
void show_matrix(double [][COL], int, int);
int guass_elimination(double *[ROW], int, int);
void exchange_row(double *[ROW], int, int);
void show_solution(double *[ROW], int, int);
void count_solution(double *[ROW],struct polar_coordinate*,struct polar_coordinate*);
/*****************************************************
***********
*********   数组计算相关函数(以上)
***********
*****************************************************/

float myatan(float,float);																																																	//相位补偿，将atan函数的值域扩为360度
void cal_m_test(struct polar_coordinate,struct polar_coordinate,struct polar_coordinate,struct polar_coordinate,						//测试使用函数
						struct polar_coordinate,struct polar_coordinate,struct polar_coordinate*,struct polar_coordinate*);
void cal_m(struct polar_coordinate,struct polar_coordinate,struct polar_coordinate,struct polar_coordinate,									//求永久配重函数(去掉试重P)
						struct polar_coordinate,struct polar_coordinate,struct polar_coordinate*,struct polar_coordinate*);
							
void cal_k(struct polar_coordinate ca11,struct polar_coordinate ca10,struct polar_coordinate cp,struct polar_coordinate *ck);//声明求影响系数函数

extern u16 flag_T_Signal;
extern u16 flag_Calculate;
extern u16 flag_print_OK;
extern float adcx1,adcx2,temp1,temp2;
extern float X[16][32];
extern float Vibration_A[LENGTH_COUNTER],Phase_a[LENGTH_COUNTER];																														 //外部变量不赋值
extern float Vibration_B[LENGTH_COUNTER],Phase_b[LENGTH_COUNTER];

float adcx;
float temp;
int x,y;

struct polar_coordinate 			P_M1,P_M2,P_P1,P_P2,P_A10,P_A20,P_A11,P_A12,P_A21,P_A22,P_K11,P_K21,P_K12,P_K22;
struct rectangular_coordinate R_M1,R_M2,R_P1,R_P2,R_A10,R_A20,R_A11,R_A12,R_A21,R_A22,R_K11,R_K21,R_K12,R_K22;

 int  main(void)
 {
		delay_init();	    	 //延时函数初始化	
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// 设置中断优先级分组2 
		EXTIX_Init();//IO初始化
		uart_init(115200);	 //串口初始化为9600
		LED_Init();		  	 //初始化与LED连接的硬件接口 
		TIM2_Cap_Init();//溢出时间为1s,(arr+1)(psc+1)/72M
		Adc_Init();
		EXT(0,8);
		
 while(1)
	  {
			cal_k(P_A10,P_A11,P_P1,&P_K11);//求取影响系数k，需要分别求取，注意求K时候需要的震动量
			cal_k(P_A20,P_A21,P_P1,&P_K21);
			cal_k(P_A10,P_A12,P_P2,&P_K12);
			cal_k(P_A20,P_A22,P_P2,&P_K22);
			
			cal_m(P_A10,P_A20,P_K11,P_K12,P_K21,P_K22,&P_M1,&P_M2);//求取永久配重
						     
			Get_RPM();//转速测量 
			
			if(flag_Calculate == 1)	//得到左右校正平面幅相值
			{
				Get_Vibration_phase_left();
				Get_Vibration_phase_right();
				flag_Calculate = 0;
			}
  	}
}

void cal_k(struct polar_coordinate pa10,struct polar_coordinate pa11,struct polar_coordinate pp,
						struct polar_coordinate *pk)//a10为原始震动，a11为加试重后的震动，pp为增加的试重，*pk为求得的影响系数。
{
	float ax,ay;

	pk->size  = sqrt(pa11.size*pa11.size+pa10.size*pa10.size-2*pa11.size*pa10.size*cos(pa11.angle-pa10.angle))/pp.size;//该方法与毛老师的数据一致
	ax = pa11.size*cos(pa11.angle)-pa10.size*cos(pa10.angle);
	ay = pa11.size*sin(pa11.angle)-pa10.size*sin(pa10.angle);
	if(ax>0)//相位补偿，将atan函数的值域扩为360度
		{
			if(ay<0) pk->angle = atan(ay/ax)-pp.angle + 3.14159*2;
			else		 pk->angle = atan(ay/ax)-pp.angle;
		}
	else pk->angle = atan(ay/ax)-pp.angle + 3.14159;	
		
		pk->angle = pk->angle/pi*180;
}

void cal_m(struct polar_coordinate pa10,struct polar_coordinate pa20,struct polar_coordinate pk11,struct polar_coordinate pk12,//计算永久配重
						struct polar_coordinate pk21,struct polar_coordinate pk22,struct polar_coordinate *pm1,struct polar_coordinate *pm2)
{
		double Receptacle[4][5]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};//想要计算的矩阵存放于此，并在row和col说明矩阵的维数
		double *Ab_pointer[ROW];
    int row=4, col=4, i, result;	
							
			//赋值数组
			Receptacle[0][0]=pk11.size*cos(pk11.angle);Receptacle[0][1]=-pk11.size*sin(pk11.angle);Receptacle[0][2]=pk12.size*cos(pk12.angle);Receptacle[0][3]=-pk12.size*sin(pk12.angle);Receptacle[0][4]=-pa10.size*cos(pa10.angle);
			Receptacle[1][0]=pk21.size*cos(pk21.angle);Receptacle[1][1]=-pk21.size*sin(pk21.angle);Receptacle[1][2]=pk22.size*cos(pk22.angle);Receptacle[1][3]=-pk22.size*sin(pk22.angle);Receptacle[1][4]=-pa20.size*cos(pa20.angle);
			Receptacle[2][0]=pk11.size*sin(pk11.angle);Receptacle[2][1]= pk11.size*cos(pk11.angle);Receptacle[2][2]=pk12.size*sin(pk12.angle);Receptacle[2][3]= pk12.size*cos(pk12.angle);Receptacle[2][4]=-pa10.size*sin(pa10.angle);
			Receptacle[3][0]=pk21.size*sin(pk21.angle);Receptacle[3][1]= pk21.size*cos(pk21.angle);Receptacle[3][2]=pk22.size*sin(pk22.angle);Receptacle[3][3]= pk22.size*cos(pk22.angle);Receptacle[3][4]=-pa20.size*sin(pa20.angle);
						
			for(i=0;i<ROW;i++)
            Ab_pointer[i] = Receptacle[i];
			
			result = guass_elimination(Ab_pointer, row, col+1);
        if(result==1){
            printf("无数个解!\n");
//            show_solution(Ab_pointer, row, col+1);
        }
        else if(result==0){
//            printf("只有一个解!\n");
//            show_solution(Ab_pointer, row, col+1);
					count_solution(Ab_pointer,pm1,pm2);
        }
        else
            printf("无解!\n");

				
}

void show_matrix(double matrix[ROW][COL], int row, int col)
{
    int i, j;

    for(i=0;i<row;i++){
        for(j=0;j<col;j++)
            printf("%-8.3f", matrix[i][j]);
        putchar('\n');
    }

    return;
}

int guass_elimination(double *matrix[ROW], int row, int col)
{
    int result, i, j, k;
    double coe;

    for(i=0;i<row-1;i++){
        exchange_row(matrix, i, row);
        if(fabs(*(matrix[i]+i))<0.00001)
            continue;
        for(j=i+1;j<row;j++){
            coe = *(matrix[j]+i) / *(matrix[i]+i);
            for(k=i;k<col;k++)
                *(matrix[j]+k) -= coe * *(matrix[i]+k);
        }
    }

    if(col-1>row)
        result = 1;
    else if(col-1==row){
        if(fabs(*(matrix[row-1]+row-1))>0.00001)
            result = 0;
        else{
            if(fabs(*(matrix[row-1]+row))>0.00001)
                result = -1;
            else
                result = 1;
        }
    }
    else{
        result = 0;
        for(i=0;i<row;i++)
            if(fabs(*(matrix[i]+col-2))<0.00001&&fabs(*(matrix[i]+col-1))>0.00001){
                result = -1;
                break;
            }
    }


    return result;
}

void exchange_row(double *matrix[ROW], int flag, int row)
{
    int i;
    double *temp;

    for(i=flag+1;i<row;i++)
        if(fabs(*(matrix[flag]+flag))<fabs(*(matrix[i]+flag))){
            temp = matrix[flag];
            matrix[flag] = matrix[i];
            matrix[i] = temp;
        }

    return;
}

void show_solution(double *matrix[ROW], int row, int col)
{
    int i, j;

    for(i=0;i<row;i++){
        for(j=0;j<col;j++)
            printf("%-8.3f", *(matrix[i]+j));
        putchar('\n');
    }

    return;
}

void count_solution(double *matrix[ROW],struct polar_coordinate *pm1,struct polar_coordinate *pm2)
{
    float x1,x2,x3,x4;
    x4=matrix[3][4]/matrix[3][3];
		x3=(matrix[2][4]-x4*matrix[2][3])/matrix[2][2];
		x2=(matrix[1][4]-matrix[1][3]*x4-matrix[1][2]*x3)/matrix[1][1];
		x1=(matrix[0][4]-x4*matrix[0][3]-x3*matrix[0][2]-x2*matrix[0][1])/matrix[0][0];
	
	pm1->angle = myatan(x1,x2);
	pm1->size	 = x1/(cos(pm1->angle));
	pm2->angle = myatan(x3,x4);
	pm2->size	 = x3/(cos(pm2->angle));
//	P_M1.angle = pm1->angle/pi*180;//转换成度数
//	P_M2.angle = pm2->angle/pi*180;
}

float myatan(float x,float y)//相位补偿，将atan函数的值域扩为360度
{
	if(x>0)
		{
			if(y<0) return (atan(y/x) + 3.14159*2);
			else		return (atan(y/x));
		}
	else return (atan(y/x) + 3.14159);
}
