#include <stdlib.h>
#include "daqmxioctrl.h"
#include <ansi_c.h>
#include <utility.h>
#include <cvirte.h>		
#include <userint.h>
#include <NIDAQmx.h>
#include "fio.h"
#include "eib7.h"
static int panelHandle;
int stop_btn_pressed =0;
////////////데이터 저장 파일 포인터
FILE *savefp1;
char testing_result_directory[1000]={NULL};
int saveing=0;
////////////////////////
char save_directory[1000]={NULL};
double torque_set1,torque_set2,torque_initial1=0,torque_initial2=0,Angle_2_initial=0;
double P_val=0.001,I_val=0.001,D_val=0.001;
double P_Angle_Val=0.001,I_Angle_Val=0.001,D_Angle_Val=0.001;
char CF[1000], Created_File_Path[1000] = { NULL }, line[1000] = { NULL };
//////////////////////////////Test VAlue///
double Speed_Volt,torque_volt,Load_Volt,Angle_1_limit_input; //Input Voltage
double speed_Ind,speed_initial,Torque1_Ind,Torque2_Ind,Angle_1_Ind,Angle_2_Ind,former_angle_2_ind,eff_Ind,ratio_Ind,timer_ind=0,Start_initial_time=0;//Output Indicator
double *torque2_indicate_pointer=0;
//////////////////////////////Task_Handle///
static TaskHandle AI_th=0,AO_th=0;
static EIB7_HANDLE eib;  //  ------------------------------->Encoder Handle
static EIB7_AXIS axis[4];  //  ------------------------------->Encoder Handle - axes array 
unsigned long ip;             /* IP address of EIB        */
unsigned long num;            /* number of encoder axes   */
char fw_version[20];          /* firmware version string  */
char s[100];                  /* buffer for console input */

unsigned short En_status;
ENCODER_POSITION pos;
/////////////////////////////
static float64 *Data=NULL,AO_Data[3]={0,0,0};  // AO_:모터_속도,모터_토크,파우더브레이크_하중 순
static uInt32 Num_of_Channels, AO_Num_of_Channels;
///////////////////////////// 파일 입출력 관련
void File_Open_Function(int mode, char *File_path_1, char *created_file_path);
//////////////////////////////PID 관련
double set_point=0,set_point2=0;
float64 OUT=0,old_OUT=0,out2=0,old_out2=0;
double error[2],error2[2];
double proportional=0,integral=0,derivative=0,proportional2=0,integral2=0,derivative2=0;
///////////////////////////////////
/*Graph*/
double Analog_Data_for_grpah[100];
int graph_i=0;
///////////////////////////////////
int timer_activation_1=0;
int testing=0;

static int32 CVICALLBACK EveryNCallback(TaskHandle taskHandle, int32 everyNsamplesEventType, uInt32 nSamples, void *callbackData);
static int32 CVICALLBACK DoneCallback(TaskHandle taskHandle, int32 status, void *callbackData);
///////////PID_Function/////////////////
float64 control_as_pid(float64* external_voltage_out,float64* external_voltage_out_old,double Error[],double* proposional_val,double* derivative_val,double* integral_val, double* setting_value,double* indicated_value);
////////////////////////////////////////
double direction_of_torque_1=1,direction_of_torque_2=1,direction_of_angle=1;
double ctrl_direction_of_speed=1,ctrl_direction_of_torque=1;
//////////////////////////////////////
double Initial_Pulse=0;
double Pulse_Per_loop=0;
double Out_speed_rpm=0;
int rpm_timer_ctrl=0;
///////////////////////////////////////
int main (int argc, char *argv[])
{
	if (InitCVIRTE (0, argv, 0) == 0)
		return -1;	/* out of memory */
	if ((panelHandle = LoadPanel (0, "fio.uir", PANEL)) < 0)
		return -1;
	DisplayPanel (panelHandle);
	/////////////////////////////////////
    SetInputMode(PANEL,PANEL_Stop_btn,0);
	SetInputMode(PANEL,PANEL_QUITBUTTON,1);
	SetInputMode(PANEL,PANEL_Test_Start_btn,1);
	GetDir(CF);
	strcat(CF,"\\Set_up_Directory.dat");
	/////////////////////////////////////------------------------->encoder
	/////////////////////////////////////
	// 여기서는 셋업파일이 있으면 넘어가고 없으면 만듭니다.//
	File_Open_Function(1,&CF,&Created_File_Path);
	/////////////////////////////////////
	NIDAQmx_NewPhysChanAICtrl(PANEL,PANEL_AI_CH,1);
	NIDAQmx_NewPhysChanAOCtrl(PANEL,PANEL_AO_CH,1);
	/////////////////////////////////////
	if(AI_th)
	DAQmxClearTask(AI_th);
	
	if(AO_th)
	DAQmxClearTask(AO_th);
	
	RunUserInterface ();
    DiscardPanel (panelHandle);
	return 0;
}

void File_Open_Function(int mode, char *File_path_1, char *created_file_path)
{
	char filename[1000]={NULL}, filename2[1000]={1000},Default_Directory[1000]={NULL};
	strcpy(Default_Directory,created_file_path);
	
	if(mode==1)
	{
		char Torque_1_Set[]={"Torque_1_: 2.35\n"},Torque_2_Set[]={"Torque_2_: 49.0\n"},P_Set[]={"PB_P_: 1.00\n"},I_Set[]={"PB_I_: 1.00\n"},D_Set[]={"PB_D_: 1.00\n"},Ratio_Set[]={"Ratio_: 1.00\n"};
		
		strcat(filename,"\\Set_up_Directory.dat");
		strcat(filename2,"\\Save_1.dat");
				
		FILE *fp1,*fp2;
		fp1=fopen(File_path_1,"r");
		if(NULL==fp1)
		{
			fp1=fopen(File_path_1,"a");
			fputs(Torque_1_Set,fp1);
			fputs(Torque_2_Set,fp1);
			
			fputs(P_Set,fp1);
			fputs(I_Set,fp1);
			fputs(D_Set,fp1);
			fputs(Ratio_Set,fp1);
			
			fclose(fp1);
			fopen(File_path_1,"r");
		}
		fgets(created_file_path,1000,fp1);		
		SetCtrlVal(PANEL,PANEL_TEXTBOX,created_file_path);
		torque_set1=atof(strstr(created_file_path,"_: ")+2);
		fgets(created_file_path,1000,fp1);		
		SetCtrlVal(PANEL,PANEL_TEXTBOX,created_file_path);		
		torque_set2=atof(strstr(created_file_path,"_: ")+2);
		fgets(created_file_path,1000,fp1);	
		SetCtrlVal(PANEL,PANEL_TEXTBOX,created_file_path);			
		P_val=atof(strstr(created_file_path,"_: ")+2);
		fgets(created_file_path,1000,fp1);		
		SetCtrlVal(PANEL,PANEL_TEXTBOX,created_file_path);		
		I_val=atof(strstr(created_file_path,"_: ")+2);
		fgets(created_file_path,1000,fp1);
		SetCtrlVal(PANEL,PANEL_TEXTBOX,created_file_path);	
		D_val=atof(strstr(created_file_path,"_: ")+2);
		fgets(created_file_path,1000,fp1);
		SetCtrlVal(PANEL,PANEL_TEXTBOX,created_file_path);	
		ratio_Ind=atof(strstr(created_file_path,"_: ")+2);
		fclose(fp1);
		
		SetCtrlVal(PANEL,PANEL_Torque_constant_1,torque_set1);
		SetCtrlVal(PANEL,PANEL_Torque_constant_2,torque_set2);
		SetCtrlVal(PANEL,PANEL_P_constant,P_val);
		SetCtrlVal(PANEL,PANEL_I_constant,I_val);
		SetCtrlVal(PANEL,PANEL_D_constant,D_val);
	}
	if(mode==2)
	{   
		char test_speed_term[]={"I_Test_Speed(rpm)\t"},Out_test_speed_term[]={"O_Test_Speed(rpm)\t"},Angle_term[]={"Angle(dgree)\t"},Torque_1_term[]={"Torque_input(Nm)\t"},Torque_2_term[]={"Torque_output(Nm)\t"},eff_term[]={"Efficiency(%)\t"},time_status_term[]={"Time\t\n"};
		FILE *fp1,*fp2;
		fp1=fopen(File_path_1,"a");
		fputs(test_speed_term,fp1);
		fputs(Out_test_speed_term,fp1);
		fputs(Angle_term,fp1);	
		fputs(Torque_1_term,fp1);	
		fputs(Torque_2_term,fp1);
		fputs(eff_term,fp1);
		fputs(time_status_term,fp1);

		fclose(fp1);
	}
}

int CVICALLBACK QuitCallback (int panel, int control, int event,
							  void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
		QuitUserInterface(1);			
		break;
	}
	return 0;
}

int CVICALLBACK Test_Start_btn (int panel, int control, int event,
								void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
		char ao_channel_name[256];
		GetCtrlVal(PANEL,PANEL_AO_CH,ao_channel_name);
		DAQmxCreateTask("",&AO_th);
		DAQmxCreateAOVoltageChan(AO_th,ao_channel_name,"",-10,10,DAQmx_Val_Volts,"");
		DAQmxGetTaskAttribute(AO_th,DAQmx_Task_NumChans,&AO_Num_of_Channels);
		///LED////////////////////////////
		SetCtrlVal(PANEL,PANEL_LED,1);
		SetCtrlVal(PANEL,PANEL_LED_2,0);
		SetCtrlVal(PANEL,PANEL_LED_8,1);
		SetCtrlVal(PANEL,PANEL_LED_9,1);
		SetCtrlVal(PANEL,PANEL_LED_10,0);		
		//////////////////////////////////
		stop_btn_pressed=0;
		SetInputMode(PANEL,PANEL_Stop_btn,1);
		SetInputMode(PANEL,PANEL_QUITBUTTON,0);
		SetInputMode(PANEL,PANEL_Test_Start_btn,0);
		SetInputMode(PANEL,PANEL_connect_btn,0);
		SetInputMode(PANEL,PANEL_disconnect_btn,0);
		/////////////////////////////
		SetCtrlAttribute(PANEL,PANEL_control_timer,ATTR_ENABLED,1);
		SetCtrlAttribute(PANEL,PANEL_PID_chart,ATTR_ENABLED,1);
		SetCtrlAttribute(PANEL,PANEL_Graph_timer,ATTR_ENABLED,1);
		///////////////////////////////
		SetInputMode(PANEL,PANEL_logging_btn,0);
		SetInputMode(PANEL,PANEL_save_btn,0);
		
		DAQmxStartTask(AO_th);		

		break;
	}
	return 0;
}

int CVICALLBACK Stop_btn (int panel, int control, int event,
						  void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
		stop_btn_pressed =1;
		OUT=0;
		old_OUT=0;
		out2=0;
		old_out2=0;
		///LED////////////////////////////
		SetCtrlVal(PANEL,PANEL_LED,0);
		SetCtrlVal(PANEL,PANEL_LED_2,1);
		SetCtrlVal(PANEL,PANEL_LED_8,1);
		SetCtrlVal(PANEL,PANEL_LED_9,0);
		SetCtrlVal(PANEL,PANEL_LED_10,0);
		SetCtrlVal(PANEL,PANEL_Angle_Limited_LED,0);
		//////////////////////////////////
		SetInputMode(PANEL,PANEL_connect_btn,1);
		SetInputMode(PANEL,PANEL_disconnect_btn,1);
		SetInputMode(PANEL,PANEL_Stop_btn,0);
		SetInputMode(PANEL,PANEL_QUITBUTTON,1);
		SetInputMode(PANEL,PANEL_Test_Start_btn,1);		
		SetCtrlAttribute(PANEL,PANEL_PID_chart,ATTR_ENABLED,0);
		//SetCtrlAttribute(PANEL,PANEL_control_timer,ATTR_ENABLED,0);
		/////////////////////////////////////////////////////////////////
		if(saveing==1)
		{
		fclose(savefp1);
		SetInputMode(PANEL,PANEL_logging_btn,0);
		SetInputMode(PANEL,PANEL_save_btn,1);
		}
		SetCtrlAttribute(PANEL,PANEL_logging_bgw,ATTR_ENABLED,0);		
		SetCtrlAttribute(PANEL,PANEL_Graph_timer,ATTR_ENABLED,0);
		SetCtrlVal(PANEL,PANEL_LED_11,0);
		///////////////////////////////////////////////////////////////// Save File 부분 종료
		SetCtrlAttribute(PANEL,PANEL_control_timer,ATTR_ENABLED,0);
		for(int i=0;i<3;i++)
		{
			AO_Data[i]=0;
		}
		DAQmxWriteAnalogF64(AO_th,1,1,10.0,DAQmx_Val_GroupByChannel,AO_Data,NULL,NULL);
		//////////////////////////////////////
		DAQmxStopTask(AO_th);
		DAQmxClearTask(AO_th);
		AO_th=0;
		if(Data)
		{
			for(int i=0;i<3;i++)
			{
				AO_Data[i]=0; 
			}
		}
		//////////////////////////////////////
		timer_ind=0;
		saveing=0;
		break;
	}
	return 0;
}


int CVICALLBACK Time_Status (int panel, int control, int event,
							 void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:

			break;
	}
	return 0;
}

void CVICALLBACK menu_exit (int menuBar, int menuItem, void *callbackData,
							int panel)
{
	QuitUserInterface(1);
}

int CVICALLBACK cw_ccw (int panel, int control, int event,
						void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:

			break;
	}
	return 0;
}

int CVICALLBACK connect_btn (int panel, int control, int event,
							 void *callbackData, int eventData1, int eventData2)
{	char channel_name[256];
	int numRead;
	switch (event)
	{
		case EVENT_COMMIT:
		///LED////////////////////////////
		SetCtrlVal(PANEL,PANEL_LED,0);
		SetCtrlVal(PANEL,PANEL_LED_2,0);
		SetCtrlVal(PANEL,PANEL_LED_8,1);
		SetCtrlVal(PANEL,PANEL_LED_9,0);
		SetCtrlVal(PANEL,PANEL_LED_10,0);		
		//////////////////////////////////
	void CheckError(EIB7_ERR error);  //////////------------------------->Encoder//Errorcheck
	EIB7GetHostIP("192.168.1.2", &ip);
    EIB7Open(ip, &eib, 5000, fw_version, sizeof(fw_version));
	EIB7GetAxis(eib, axis, 4, &num);
	EIB7InitAxis(axis[0],EIB7_IT_Incremental,EIB7_EC_Linear,EIB7_RM_None,         /* reference marks not used */
              0,                    /* reference marks not used */
              0,                    /* reference marks not used */
              EIB7_HS_None,
              EIB7_LS_None,
              EIB7_CS_CompActive,   /* signal compensation on   */
              EIB7_BW_High,         /* signal bandwidth: high   */
              EIB7_CLK_Default,     /* not used for incremental interface */
              EIB7_RT_Long,         /* not used for incremental interface */
              EIB7_CT_Long          /* not used for incremental interface */
              );
		/////////////////*ANALOG_INPUT_CONFIG*//////////////////////////////////////////	
		GetCtrlVal(PANEL,PANEL_AI_CH,channel_name);

		DAQmxCreateTask("",&AI_th);
		DAQmxCreateAIVoltageChan(AI_th,channel_name,"",DAQmx_Val_Cfg_Default,-10,10,DAQmx_Val_Volts,NULL);
		DAQmxCfgSampClkTiming(AI_th,"",1000,DAQmx_Val_Rising,DAQmx_Val_ContSamps,10);
		DAQmxGetTaskAttribute(AI_th,DAQmx_Task_NumChans,&Num_of_Channels);
		
		
		DAQmxRegisterEveryNSamplesEvent(AI_th,DAQmx_Val_Acquired_Into_Buffer,10,0,EveryNCallback,NULL);
		DAQmxRegisterDoneEvent(AI_th,0,DoneCallback,NULL);
		DAQmxStartTask(AI_th);		

		Data=(float64*)malloc(10*Num_of_Channels*sizeof(float64));
		SetCtrlAttribute(PANEL,PANEL_TIMER,ATTR_ENABLED,1);		
		/////////////////*ANALOG_OUTPUT_CONFIG*//////////////////////////////////////////

		/*
		timer_activation_1=1;
		while(1)
		{
		
		if(testing==1)
		break;
		
		//DAQmxReadAnalogF64(AI_th,1,10.0,DAQmx_Val_GroupByScanNumber,Data,1*Num_of_Channels,&numRead,NULL);

		if(timer_activation_1==1)
		{
		//SetCtrlAttribute(PANEL,PANEL_TIMER,ATTR_ENABLED,1);
		//SetCtrlAttribute(PANEL,PANEL_Speed_ind_timer,ATTR_ENABLED,1);
		}
		ProcessSystemEvents();
		}*/
		testing=0;
		timer_activation_1=0;
		break;
	}
	return 0;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int32 CVICALLBACK EveryNCallback(TaskHandle taskHandle, int32 everyNsamplesEventType, uInt32 nSamples, void *callbackData)
{
	int32   error=0;
	char    errBuff[2048]={'\0'};
	int     numRead;
	/*********************************************/
	// DAQmx Read Code
	/*********************************************/
	DAQmxReadAnalogF64(taskHandle,10,10.0,DAQmx_Val_GroupByScanNumber,Data,10*Num_of_Channels,&numRead,NULL);
	//if( numRead>0 )
	return 0;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int32 CVICALLBACK DoneCallback(TaskHandle taskHandle, int32 status, void *callbackData)
{
	int32   error=0;
	char    errBuff[2048]={'\0'};

	if( Data ) {
		free(Data);
		Data = NULL;
	}
	AI_th = 0;
	// Check to see if an error stopped the task.
	return 0; 
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int CVICALLBACK disconnect_btn (int panel, int control, int event,
								void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
		testing=1;
		///LED////////////////////////////
		SetCtrlVal(PANEL,PANEL_LED,0);
		SetCtrlVal(PANEL,PANEL_LED_2,0);
		SetCtrlVal(PANEL,PANEL_LED_8,0);
		SetCtrlVal(PANEL,PANEL_LED_9,0);
		SetCtrlVal(PANEL,PANEL_LED_10,0);		
		//////////////////////////////////
		
		EIB7Close(eib);//-------------->Encoder_Stop
		
		SetCtrlAttribute(PANEL,PANEL_TIMER,ATTR_ENABLED,0);
		SetCtrlAttribute(PANEL,PANEL_rpm_timer,ATTR_ENABLED,0);
		rpm_timer_ctrl=0;
		SetCtrlAttribute(PANEL,PANEL_control_timer,ATTR_ENABLED,0);
		for(int i=0;i<3;i++)
		{
			AO_Data[i]=0; 
		}
		DAQmxWriteAnalogF64(AO_th,1,1,10.0,DAQmx_Val_GroupByChannel,AO_Data,NULL,NULL);	
		DAQmxStopTask(AI_th);
		DAQmxClearTask(AI_th);
		AI_th=0;
		DAQmxStopTask(AO_th);
		DAQmxClearTask(AO_th);
		AO_th=0;
		if(Data)
		{
			free(Data);
			Data=NULL; 
		}
		if(Data)
		{
			free(AO_Data);
			for(int i=0;i<3;i++)
			{
				AO_Data[i]=0; 
			}
		}
		break;
	}
	return 0;
}

int CVICALLBACK timer_1 (int panel, int control, int event,
						 void *callbackData, int eventData1, int eventData2)
{
	int numRead;
	switch (event)
	{
		case EVENT_TIMER_TICK:
		int D_A=1,D_IT=1,D_OT=1,D_A_Check=0,D_IT_Check=0,D_OT_Check=0;
		void check_direciton_check_box();
		/////////////////////////////////////////////////////////////
		SetCtrlAttribute(PANEL,PANEL_rpm_timer,ATTR_ENABLED,1);	
		GetCtrlVal(PANEL,PANEL_Convert_Check_3,&D_A_Check);
		GetCtrlVal(PANEL,PANEL_Convert_Check_1,&D_IT_Check);
		GetCtrlVal(PANEL,PANEL_Convert_Check_2,&D_OT_Check);
		/////////////////////////////////////////////////////////////
		if(D_A_Check==1)
		D_A=-1;
		if(D_IT_Check==1)
		D_IT=-1;
		if(D_OT_Check==1)
		D_OT=-1;
		/////////////////////////////////////////////////////////////
		/////////////////////////////////////////////////////////////
		EIB7GetPosition(axis[0], &En_status, &pos);
		Angle_1_Ind=pos;
		Angle_2_Ind=direction_of_angle*(Angle_1_Ind/67108864*360)*D_A;
		SetCtrlVal(PANEL,PANEL_Angle_Ind,(Angle_2_Ind-Angle_2_initial));
		/////////////////////////////////////////////////////////////
		speed_Ind=((Data[2]-speed_initial)*377);
		Torque1_Ind=((Data[0]*torque_set1)-torque_initial1)*D_IT;
		Torque2_Ind=((Data[1]*torque_set2)-torque_initial2)*D_OT;
		eff_Ind=Torque2_Ind/(Torque1_Ind*ratio_Ind)*100;
		/////////////////////////////////////////////////////////////
		
		/////////////////////////////////////////////////////////////
		SetCtrlVal(PANEL,PANEL_Speed_Ind,speed_Ind);
		SetCtrlVal(PANEL,PANEL_I_Torque_Ind,Torque1_Ind);
		SetCtrlVal(PANEL,PANEL_O_Torque_Ind,Torque2_Ind);
		SetCtrlVal(PANEL,PANEL_Eff_Ind,eff_Ind);
		/////////////////////////////////////////////////////////////
		break;
	}
	return 0;
}

int CVICALLBACK control_timer (int panel, int control, int event, void *callbackData, int eventData1, int eventData2)
{
	double angle_control_val,angle_limit_val;
	int angle_limitcheck_val,CW_speed_Vout=1,CW_Torque_Vout=1,CW_CCW_Option=0;
	switch (event) 
	{
		case EVENT_TIMER_TICK:		
		
		timer_ind+=0.01;
		SetCtrlVal(PANEL,PANEL_Start_Timer,timer_ind);
		////////////////////////////////////////////
		if(stop_btn_pressed==0)
		GetCtrlVal(PANEL,PANEL_NUMERIC_5,&set_point);
		else
		set_point=0.0;
		///////////////////////////////////////////
		//AO_Data[2]=control_as_pid(OUT,old_OUT,&error,proportional,derivative,integral,P_val,D_val,I_val,set_point,Torque2_Ind);
		AO_Data[2]=control_as_pid(&OUT,&old_OUT,error,&P_val,&D_val,&I_val,&set_point,&Torque2_Ind);
		///////////////////////////////////////////
		GetCtrlVal(PANEL,PANEL_cw_ccw,&CW_CCW_Option);
		if(CW_CCW_Option==1)
		{
			CW_speed_Vout=-1;
			CW_Torque_Vout=-1;
		}			
		///////////////////////////////////////////
		GetCtrlVal(PANEL,PANEL_NUMERIC_2,&Speed_Volt);
		AO_Data[1]=(Speed_Volt/((double)200.0))*CW_speed_Vout;
		GetCtrlVal(PANEL,PANEL_NUMERIC_3,&torque_volt);
		torque_volt=torque_volt*0.47904*CW_Torque_Vout;
		AO_Data[0]=(double)(torque_volt);
		DAQmxWriteAnalogF64(AO_th,1,1,10.0,DAQmx_Val_GroupByChannel,AO_Data,NULL,NULL);
		GetCtrlVal(PANEL,PANEL_Angle_Limit_check,&angle_limitcheck_val);
		//////////////////Check_Box/////////////////////////
		if(angle_limitcheck_val==1)
		{	
			GetCtrlVal(PANEL,PANEL_NUMERIC_4,&angle_limit_val);
			GetCtrlVal(PANEL,PANEL_Angle_Ind,&angle_control_val);
			if(angle_control_val>angle_limit_val)
			{
			SetCtrlVal(PANEL,PANEL_Angle_Limited_LED,1);
			SetCtrlAttribute(PANEL,PANEL_control_timer,ATTR_ENABLED,0);
			for(int i=0;i<3;i++)
			{
				AO_Data[i]=0;
			}
			DAQmxWriteAnalogF64(AO_th,1,1,10.0,DAQmx_Val_GroupByChannel,AO_Data,NULL,NULL);
			}
		}
		////////////////////////////////////////////////////
		break;
	}
	return 0;
}

int CVICALLBACK PID_chart (int panel, int control, int event,
						   void *callbackData, int eventData1, int eventData2)
{double torque_graph[2];
 double angle_stripchart_data;
	switch (event)
	{
		case EVENT_TIMER_TICK:
		/*fprintf(savefp1,"%lf\t",speed_Ind);
		fprintf(savefp1,"%lf\t",Angle_2_Ind-Angle_2_initial);
		*/
		torque_graph[0]=Torque1_Ind;
		torque_graph[1]=Torque2_Ind;
		angle_stripchart_data=(Angle_2_Ind-Angle_2_initial);
			//GetCtrlVal(PANEL,PANEL_I_Torque_Ind,torque_graph);
			//GetCtrlVal(PANEL,PANEL_O_Torque_Ind,&torque_graph[1]);
		PlotStripChart(PANEL,PANEL_STRIPCHART_3,&speed_Ind,1,0,0,VAL_DOUBLE);
		PlotStripChart(PANEL,PANEL_STRIPCHART_4,&angle_stripchart_data,1,0,0,VAL_DOUBLE);
		PlotStripChart(PANEL,PANEL_STRIPCHART,torque_graph,2,0,0,VAL_DOUBLE);
		break;
	}
	return 0;
}

int CVICALLBACK T_Zero_set_btn (int panel, int control, int event, // Torque Zero Set!!!!
								void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
		GetCtrlVal(PANEL,PANEL_I_Torque_Ind,&torque_initial1);
		GetCtrlVal(PANEL,PANEL_O_Torque_Ind,&torque_initial2);
		speed_initial=Data[2];
		//GetCtrlVal(PANEL,PANEL_Speed_Ind,&speed_initial);
		break;
	}
	return 0;
}

int CVICALLBACK Angle_Zero_set_btn (int panel, int control, int event,
									void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
		Angle_2_initial=Angle_2_Ind;
			break;
	}
	return 0;
}


int CVICALLBACK calculate_timer_1 (int panel, int control, int event,
								   void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_TIMER_TICK:

			break;
	}
	return 0;
}

int CVICALLBACK logging_btn (int panel, int control, int event,
							 void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
		savefp1=fopen(testing_result_directory,"a");
		SetCtrlAttribute(PANEL,PANEL_logging_bgw,ATTR_ENABLED,1);
		SetInputMode(PANEL,PANEL_save_btn,0);
		SetInputMode(PANEL,PANEL_logging_btn,0);
		break;
	}
	return 0;
}

int CVICALLBACK save_btn (int panel, int control, int event,
						  void *callbackData, int eventData1, int eventData2)
{

	int ret;
	switch (event)
	{
		case EVENT_COMMIT:
		GetDir(CF);
		ret=FileSelectPopup("","*.dat","*.dat","Save Data File",VAL_SAVE_BUTTON,0,0,1,0,testing_result_directory);
		if(ret>0)
		SetCtrlVal(PANEL,PANEL_Save_File_Dir,testing_result_directory);

		File_Open_Function(2,&testing_result_directory,&Created_File_Path);
		saveing=1;  //Save 파일 상태 확인
		//////////////////////////////////////////////////
		SetInputMode(PANEL,PANEL_logging_btn,1);
		SetInputMode(PANEL,PANEL_save_btn,0);
		break;
	}
	return 0;
}


int CVICALLBACK logging_bgw (int panel, int control, int event,
							 void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_TIMER_TICK:

		fprintf(savefp1,"%lf\t",speed_Ind);
		fprintf(savefp1,"%lf\t",Out_speed_rpm);
		fprintf(savefp1,"%lf\t",Angle_2_Ind-Angle_2_initial);
		fprintf(savefp1,"%lf\t",Torque1_Ind);
		fprintf(savefp1,"%lf\t",Torque2_Ind);
		fprintf(savefp1,"%lf\t",eff_Ind);
		fprintf(savefp1,"%lf\n",timer_ind);
		SetCtrlVal(PANEL,PANEL_LED_11,1);
		
		break;
	}
	return 0;
}


	int CVICALLBACK Graph_timer (int panel, int control, int event,
								 void *callbackData, int eventData1, int eventData2)
	{
		switch (event)
		{ //eff_Ind
			case EVENT_TIMER_TICK:
			
			if(graph_i>99)
			{
				for(int move_data=0;move_data<99;move_data++)
				Analog_Data_for_grpah[move_data]=Analog_Data_for_grpah[move_data+1];
				Analog_Data_for_grpah[99]= eff_Ind;
				graph_i=99;
			}
			else
			{
				Analog_Data_for_grpah[graph_i]=eff_Ind;
				graph_i++;
			}
			RefreshGraph(PANEL,PANEL_GRAPH);
			DeleteGraphPlot(PANEL,PANEL_GRAPH,-1,1);
			PlotY(PANEL,PANEL_GRAPH,Analog_Data_for_grpah,graph_i,VAL_DOUBLE,VAL_FAT_LINE,VAL_EMPTY_SQUARE,VAL_SOLID,1,VAL_RED);


			break;
		}
		return 0;
	}

float64 control_as_pid(float64* external_voltage_out,float64* external_voltage_out_old,double Error[],double* proposional_val,double* derivative_val,double* integral_val, double* setting_value,double* indicated_value)
{
	double proposional_data_fn=0;
	double integral_data_fn=0;
	double derivative_data_fn=0;
		
	Error[0]=*setting_value-*indicated_value;//*setting_val_fn-*indicated_value_fn;
	proposional_data_fn=*proposional_val*Error[0];
	integral_data_fn=*integral_val*(Error[0]+Error[1]);
	derivative_data_fn=*derivative_val*(Error[0]-Error[1]);

	*external_voltage_out=proposional_data_fn+integral_data_fn+derivative_data_fn+*external_voltage_out_old;
	Error[1]=Error[0];
	
	*external_voltage_out=(*external_voltage_out>10)?10:*external_voltage_out;//출력의 정규화
	*external_voltage_out=(*external_voltage_out<-10)?-10:*external_voltage_out;
	
	//MessagePopup("","");
	*external_voltage_out_old=*external_voltage_out;//이전 출력을 저장

	return *external_voltage_out;
}

int CVICALLBACK CCW_Movement_btn (int panel, int control, int event,
								  void *callbackData, int eventData1, int eventData2)
{
	int active_CCW=0;
	int xcoord,ycoord,leftbtn,rightbtn,keys;
	char ao_channel_name[256];
	switch (event)
	{
		case EVENT_LEFT_CLICK:

		GetCtrlVal(PANEL,PANEL_AO_CH,ao_channel_name);
        DAQmxCreateTask("",&AO_th);
        DAQmxCreateAOVoltageChan(AO_th,ao_channel_name,"",-10,10,DAQmx_Val_Volts,"");
        DAQmxGetTaskAttribute(AO_th,DAQmx_Task_NumChans,&AO_Num_of_Channels);
        //////////////////////////////////
        stop_btn_pressed=0;
        /////////////////////////////

        SetCtrlAttribute(PANEL,PANEL_PID_chart,ATTR_ENABLED,1); //
        SetCtrlAttribute(PANEL,PANEL_Graph_timer,ATTR_ENABLED,1); //
        DAQmxStartTask(AO_th);
		do{
			SetCtrlVal(PANEL,PANEL_Manual_LED,1);
			GetGlobalMouseState(NULL,&xcoord, &ycoord, &leftbtn, &rightbtn, &keys);
			GetCtrlVal(PANEL,PANEL_move_button_speed,&Speed_Volt);
            AO_Data[1]=Speed_Volt/((double)200.0);
            GetCtrlVal(PANEL,PANEL_move_button_Torque,&torque_volt);
            torque_volt=torque_volt*0.47904;
            AO_Data[0]=(double)(torque_volt);
            DAQmxWriteAnalogF64(AO_th,1,1,10.0,DAQmx_Val_GroupByChannel,AO_Data,NULL,NULL);

			ProcessSystemEvents();
			Delay(0.1);
		}while(leftbtn==1);
		SetCtrlVal(PANEL,PANEL_Manual_LED,0);
		//MessagePopup("","");
		stop_btn_pressed =1;
		SetCtrlAttribute(PANEL,PANEL_PID_chart,ATTR_ENABLED,0);
        /////////////////////////////////////////////////////////////////                 
        SetCtrlAttribute(PANEL,PANEL_Graph_timer,ATTR_ENABLED,0);
        ///////////////////////////////////////////////////////////////// Save File 부분 종료

         for(int i=0;i<3;i++)
          {
             AO_Data[i]=0;
          }
           DAQmxWriteAnalogF64(AO_th,1,1,10.0,DAQmx_Val_GroupByChannel,AO_Data,NULL,NULL);

           DAQmxStopTask(AO_th);
            DAQmxClearTask(AO_th);
           AO_th=0;
         if(Data)
         {
           for(int i=0;i<3;i++)
              {
                   AO_Data[i]=0;
              }
         }

         break;
	}
	return 0;
}

int CVICALLBACK CW_Movement_btn (int panel, int control, int event,
								 void *callbackData, int eventData1, int eventData2)
{
	int active_CCW=0;
	int xcoord,ycoord,leftbtn,rightbtn,keys;
	char ao_channel_name[256];
	switch (event)
	{
		case EVENT_COMMIT:		case EVENT_LEFT_CLICK:

		GetCtrlVal(PANEL,PANEL_AO_CH,ao_channel_name);
        DAQmxCreateTask("",&AO_th);
        DAQmxCreateAOVoltageChan(AO_th,ao_channel_name,"",-10,10,DAQmx_Val_Volts,"");
        DAQmxGetTaskAttribute(AO_th,DAQmx_Task_NumChans,&AO_Num_of_Channels);
        //////////////////////////////////
        stop_btn_pressed=0;
        /////////////////////////////

        SetCtrlAttribute(PANEL,PANEL_PID_chart,ATTR_ENABLED,1); //
        SetCtrlAttribute(PANEL,PANEL_Graph_timer,ATTR_ENABLED,1); //
        DAQmxStartTask(AO_th);
		do{
			SetCtrlVal(PANEL,PANEL_Manual_LED,1);
			GetGlobalMouseState(NULL,&xcoord, &ycoord, &leftbtn, &rightbtn, &keys);
			GetCtrlVal(PANEL,PANEL_move_button_speed,&Speed_Volt);
            AO_Data[1]=(Speed_Volt/((double)200.0))*1;
            GetCtrlVal(PANEL,PANEL_move_button_Torque,&torque_volt);
            torque_volt=torque_volt*0.47904;
            AO_Data[0]=(double)(torque_volt)*-1;
            DAQmxWriteAnalogF64(AO_th,1,1,10.0,DAQmx_Val_GroupByChannel,AO_Data,NULL,NULL);

			ProcessSystemEvents();
			Delay(0.1);
		}while(leftbtn==1);
		SetCtrlVal(PANEL,PANEL_Manual_LED,0);
		//MessagePopup("","");
		stop_btn_pressed =1;
		SetCtrlAttribute(PANEL,PANEL_PID_chart,ATTR_ENABLED,0);
        /////////////////////////////////////////////////////////////////                 
        SetCtrlAttribute(PANEL,PANEL_Graph_timer,ATTR_ENABLED,0);
        ///////////////////////////////////////////////////////////////// Save File 부분 종료

         for(int i=0;i<3;i++)
          {
             AO_Data[i]=0;
          }
           DAQmxWriteAnalogF64(AO_th,1,1,10.0,DAQmx_Val_GroupByChannel,AO_Data,NULL,NULL);

           DAQmxStopTask(AO_th);
            DAQmxClearTask(AO_th);
           AO_th=0;
         if(Data)
         {
           for(int i=0;i<3;i++)
              {
                   AO_Data[i]=0;
              }
         }

         break;
	}
	return 0;
}

int CVICALLBACK RPM_timer (int panel, int control, int event,
						   void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_TIMER_TICK:
		if(rpm_timer_ctrl==1)
		{
			Pulse_Per_loop=Angle_1_Ind-Initial_Pulse;
			
			if(Pulse_Per_loop<0)
			Pulse_Per_loop=Pulse_Per_loop*(-1);
			
			SetCtrlVal(PANEL,PANEL_pulse_1,Angle_1_Ind);			
			
			SetCtrlVal(PANEL,PANEL_pulse_2,Pulse_Per_loop);
			Out_speed_rpm=(Pulse_Per_loop*60)/((67108864));
			SetCtrlVal(PANEL,PANEL_O_Speed_Ind,Out_speed_rpm);
		}
		Initial_Pulse=Angle_1_Ind;		
		rpm_timer_ctrl=1;
		break;
	}
	return 0;
}
