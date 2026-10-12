#pragma  once
#ifndef __LHROBOT_API_H__
#define __LHROBOT_API_H__
#ifdef LHAPI
#define LHAPI extern "C" _declspec(dllimport) 
#else
#define LHAPI extern "C" _declspec(dllexport) 
#endif

#define CMD_MODE_IS_DEFAULT 0     /*Robot is not in Interpreter or API mode    */
#define CMD_MODE_IS_INTERPRETER 1 /*Robot is in Interpreter mode               */
#define CMD_MODE_IS_API 2         /*Robot is in API mode                       */

#define ETH_CTRL_CMD_ERR 1
#define ETH_CTRL_ENABLE_ERR 2
#define ETH_CTRL_EMPTY_CMD_ERR 3


#define ETH_CTRL_ERROR_STATUS_ERR 8
#define ETH_CTRL_PARAMETER_ERR 9
#define ETH_CTRL_MODE_ERR 10
#define ETH_CTRL_PARM_COUNT_ERR 11
#define ETH_CTRL_PARM_TYPE_ERR 12
#define ETH_CTRL_CMD_EXECUTE_ERR 13

#define ETH_CTRL_CMD_MODE_ERR 14

#define ETH_CTRL_OUT_INDEX_ERROR 15
#define ETH_CTRL_OUT_TOOL_ERROR 16

#define ETH_CTRL_INTERNAL_ERR 100

#define TRIGGER_MODE_IS_DEFAULT_NO_SIGNAL 0/*No trigger signal is required, if there are motion data, robot will directly start to run*/
#define TRIGGER_MODE_SOFTWARE 1            /*Have motion data and need software trigger signal to start running                      */
#define TRIGGER_MODE_HARDWARE 2            /*Have motion data and need hardware trigger signal to start running                      */

typedef struct _SPEED {
	double vel;  /*movement velocity	                                                       */
	double accel;/*movement acceleration	                                                   */
	double decel;/*movement deceleration	                                                   */
	double tvel; /*movement Compliance coefficient, take the float value of 0 ~1               */
	double rvel; /*The retention parameters won't be valid, it is recommended to fill in 0     */
	int    blend;/*0:No transition effect, 1~100 add transition effect                         */
	double leave;/*The retention parameters won't be valid, it is recommended to fill in 0	   */
	double reach;/*The retention parameters won't be valid, it is recommended to fill in 0     */
}SPEED;


typedef struct  _DHPARAMETER {
	float L1;
	float L2;
	float L3;
	float D1;
	float D2;
	float D3;
}DH;


typedef enum _REMOTE_MODE {
	sys_REMOTE_IO = 0x02,           /*IO*/
	sys_REMOTE_MODBUS_RTU = 0x03,   /*RTU*/
	sys_REMOTE_MODBUS_TCP = 0x04,   /*TCP*/
	sys_REMOTE_SERIAL = 0x05,       /*串口*/
	sys_REMOTE_NET = 0x06,          /*网络*/
	sys_REMOTE_API = 0x07,          /*api*/
}RemoteControlMode;


typedef enum
{
	sys_RUN = 0x00,   /*停止状态*/
	sys_PAUSE = 0x01,
	sys_STOP = 0x02,   /*停止状态*/
	sys_STANDSTILL = 0x03,
	sys_ERROR = 0x04,
} SysState;

typedef enum _TRIGGER_MODE {
	NO_SIGNAL = 0x00,	/*Robot is in NO_SIGNAL mode */
	SOFTWARE = 0x01,	/*Robot is SOFTWARE mode     */
	HARDWARE = 0x02,	/*Robot is in HARDWEAR mode	 */
}TRIGGER_MODE;





typedef enum _STEPMODE {
	WORLD = 0x00,	/*Robot is in WORLD coordinate	*/
	JOINT = 0x01,	/*Robot is in JOINT coordinate	*/
	USER = 0x02,	/*Robot is in USER coordinate	*/
	TOOL = 0x03,	/*Robot is in TOOL coordinate	*/
}STEPMODE;

typedef struct _LPOSITION {
	double X;
	double Y;
	double Z;
	double U;
	double V;
	double W;
	double R;	/*Additional axis R				                                          */
	double S;	/*Additional axis S				                                          */
	double T;	/*The retention parameters won't be valid, it is recommended to fill in 0 */
	int Local;	/*User coordinate index				                                      */
	int Hand;	/*Right-handed system or left-handed system                               */
	int Elbow;	/*Elbow posture	up or down			                                      */
	int Wrist;	/*Wrist posture flip or noflip		                                      */
	int J1Flag;	/*For ceiling mounting robot, default 0	                                  */
	int J2Flag; /*For ceiling mounting robot, default 0	                                  */
	int J4Flag; /*For 6-Axis robot default 0                                              */
	int J6Flag; /*For 6-Axis robot default 0                                              */
	int J1Angle;/*The retention parameters won't be valid, it is recommended to fill in 0 */
	int J4Angle;/*The retention parameters won't be valid, it is recommended to fill in 0 */
}LPOSITION;

typedef struct _POSITION_OFFSET {
	double dX; /*dX: Offset distance in X direction, for example: current x is 5, and the dX= 5, then the target position x is 10*/
	double dY; /*dY: Offset distance in Y direction, for example: current y is 5, and the dY= 5, then the target position y is 10*/
	double dZ; /*dZ: Offset distance in Z direction, for example: current z is 5, and the dZ= 5, then the target position z is 10*/
	double dU;
	double dV;
	double dW;
	unsigned int local;
}POSITION_OFFSET;

typedef struct _FILEINFO {
	char filename[128];
	int isize;
}FILEINFO;

typedef struct _JOINTRANGE
{
	float J1_Min;
	float J2_Min;
	float J3_Min;
	float J4_Min;
	float J5_Min;
	float J6_Min;
	float J1_Max;
	float J2_Max;
	float J3_Max;
	float J4_Max;
	float J5_Max;
	float J6_Max;
}JOINTRANGE;


typedef struct _CAR_POS 
{//lfs-2021-0630
	double  X;
	double  Y;
	double  Z;
	double  U;
	double  V;
	double  W;
}CAR_POS;

typedef enum _SYS_CONTROL_MODE{
	sys_TEACH = 0x00,   /*示教模式*/
	sys_AUTO = 0x01,    /*自动模式*/
	sys_EX_AUTO = 0x02,    /*远程模式*/
	sys_EX_TEACH = 0x03,    /*远程示教模式*/
}SysControlMode;



typedef struct _LHSYSTEMTIME
{
	unsigned int Year;
	unsigned int Month;
	unsigned int Day;
	unsigned int Hour;
	unsigned int Minute;
	unsigned int Second;
}LHSYSTEMTIME;

typedef struct _BOX_RANGE
{
	float minX;
	float maxX;
	float minY;
	float maxY;
	float minZ;
	float maxZ;
}BOXRANGE;

typedef enum _OUTPUTMODE {
	INSIDE_OFF = 0x00,	/*Open the output port after leaving the area  */
	INSIDE_ON = 0x01    /*Open the output port after entering the area */
}OUTPUTMODE;

typedef struct _BOX_INFO
{
	float minX;
	float maxX;
	float minY;
	float maxY;
	float minZ;
	float maxZ;
	int outputIndex;
	OUTPUTMODE outputMode;
	unsigned int boxEnable;
}BOXINFO;


typedef struct _JPOSITION {
	double J1;
	double J2;
	double J3;
	double J4;
	double J5;
	double J6;
}JPOSITION;

typedef struct _CAR_MOTION_PARAMETER {
	int speed;
	int acceleration;
	int jerk;
	int pose_speed;
	int pose_acceleration;
	int pose_jerk;
}CAR_MOTION_PARAMETER;

typedef struct _TOOL_BOX_INFO {
	unsigned int nEnable;
	unsigned int nType; /* 0-圆柱体  1-双圆柱体(必须要有tool2)*/
	float fZ1;
	float fH1;
	float fR1;
	unsigned int nTool1;
	unsigned int nTool2;
}TOOL_BOX_INFO;

typedef struct _BODY_BOX {
	unsigned int enable;
	float R1;
	float R2;
	float R3;
	float Z1;
	float Z2;
	float Z3;
	float Z4;
}BODY_BOX;

typedef struct _JOINT_MOTION_PARAMETER {
	int MaxSpeed[6];
	int MaxAcceleration[6];
	int Jerk[6];
}JOINT_MOTION_PARAMETER;

typedef struct _REDUCTION_RATIO
{
	float Jnt1;
	float Jnt2;
	float Jnt3;
	float Jnt4;
	float Jnt5;
	float Jnt6;
}REDUCTION_RATIO;


typedef struct _PROCESS_OUTPUT {
	unsigned int outIndex;
	bool  outFlag;
	unsigned int outMode;
	unsigned int outPercent;
	unsigned int outFunction;
}PROCESS_OUTPUT;

typedef struct _LHJPOSITION {
	float J1;
	float J2;
	float J3;
	float J4;
	float J5;
	float J6;
	float R;
	float S;
	float T;
}LHJPOSITION;

typedef enum _TASK_STATUS {
	task_RUN = 0x00,          /*在运行中*/
	task_STOP = 0x01,         /*停止*/
	task_PAUSE = 0x02,        /*暂停*/
	task_ERROR = 0x03,        /*错误*/
}TaskStatus;

/**
Connect robot
return value: If success, the return value is 0
**/
LHAPI int   connectRobot(const char *IP, const char *PORT);

/**
Disconnect robot
return value: If success, the return value is 0
**/
LHAPI int   disconnect();

/**
Import project file to controller
prjPath     : The path of the project file to be imported, for example: d:\\test
fileName    : The name of the project file to be imported, for example: Main.lh
return value: If success,the return value is 0
**/
LHAPI int  importProject(const char *prjPath, const char *fileName);


/**
Export project file from controller
prjPath     : The storage path of the project file, for example: d:\\test
fileName    : The name of the file that you want to import from controller, for example: Main.lh
return value: If success,the return value is 0
**/
LHAPI int  exportProject(const char *prjPath, const char *fileName);

/**
Get project file name list from controller
fileListArray  : The structure arry that contains the file list name and size
iarraySize     : The maximum number of files can be obtained
ireturnSize    : The number of files currently exist in the controller
**/

LHAPI int  getRemoteFileList(FILEINFO *fileListArray, unsigned int iarraySize, unsigned int *ireturnSize);

/**
Robot servo on
return value: If success, the return value is 0
**/
LHAPI int 	ServoOn();

/**
Robot servo off
return value: If success, the return value is 0
**/
LHAPI int 	ServoOff();

/**
Get robot servo status
return value: If success, the return value is 0
**/
LHAPI int 	getServoStatus(unsigned int *servoStatus);

/**
Run the robot program "Main.lh" in INTERPRETER mode
return value    : If success, the return value is 0
**/
LHAPI int   start();

/**
Stop the robot program in INTERPRETER mode when it has been run
return value    : If success, the return value is 0
**/
LHAPI int   stop();

/**
Pause the robot program in INTERPRETER mode when it has been run
return value    : If success, the return value is 0
**/
LHAPI int   pause();

/**
Resume program running in INTERPRETER mode when it has been paused
return value    : If success, the return value is 0
**/
LHAPI int   resume();

/**
Clear robot fault
return value: If success, the return value is 0
**/


LHAPI int   clearFault();

/**
Reboot robot
return value: If success, the return value is 0
**/
LHAPI int   reboot();

/**
Get error code of the robot
errorcode   : Error code of the robot
iSize       : The size of error code
return value: If success, the return value is 0
**/
LHAPI int 	getRobotErrorCode(char *errorcode, unsigned int iSize);

/**
Get robot status
state       : Enumeration variable representing the status of the robot system
return value: If success, the return value is 0
**/
LHAPI int   getRobotStatus(SysState *state);


//LHAPI int   getRobotSysCtrlMode(SysControlMode *sysMode);

/**
Joint motion by point name
pointName: Target point name
toolIndex: The index of the tool coordinate
speed	 : The speed parameter
outIndex : Trigger the output signal when the robot motion is finished
return   : If success, the return value is 0
**/
LHAPI int 	MOVJ(const char *pointName, unsigned int toolIndex, SPEED speed, unsigned int outIndex);

/**
Straight line motion by specified point name
pointName: Target point name
toolIndex: The index of the tool coordinate
speed	 : The speed parameter
outIndex : Trigger the output signal when the robot motion is finished
return   : If success, the return value is 0
**/
LHAPI int 	MOVL(const char *pointName, unsigned int toolIndex, SPEED speed, unsigned int outIndex);

/**
Jump motion by a specified point name
pointName: Target point name
zLimit   : Horizontal height of the JUMP motion, which is Z-Axis coordinate height
toolIndex: The index of the tool coordinate
speed	 : The speed parameter
outIndex : Trigger the output signal when the robot motion is finished
return   : If success, the return value is 0
**/
LHAPI int 	JUMP(const char *pointName, float zLimit, unsigned int toolIndex, SPEED speed, unsigned int outIndex);

/**
Joint motion by a specified LPOSITION structure that contains the point information
point	 : The structure that stored the information of the target point
toolIndex: The index of the tool coordinate
speed	 : The speed parameter
outIndex : Trigger the output signal when the robot motion is finished
return   : If success, the return value is 0
**/
LHAPI int 	MOVJP(LPOSITION point, unsigned int toolIndex, SPEED speed, unsigned int outIndex);

/**
Straight line motion by a specified LPOSITION structure that contains the point information
point	 : The structure that stored the information of the target point
toolIndex: The index of the tool coordinate
speed	 : The speed parameter
outIndex : Trigger the output signal when the robot motion is finished
return   : If success, the return value is 0
**/
LHAPI int 	MOVLP(LPOSITION point, unsigned int toolIndex, SPEED speed, unsigned int outIndex);

/**
Jump motion by a specified LPOSITION structure that contains the point information
point	 : The structure that stored the information of the target point
zLimit   : Horizontal height of the JUMP motion, which is Z-Axis coordinate height
toolIndex: The index of the tool coordinate
speed    : The speed parameter
outIndex : Trigger the output signal when the robot motion is finished
return   : If success, the return value is 0
**/
LHAPI int 	JUMPP(LPOSITION point, float zLimit, unsigned int toolIndex, SPEED speed, unsigned int outIndex);

/**
Move to the target joint angle position in a point-to-point motion
J1~J6   : Target joint angle, which is the joint coordinate value
R, S, T : Additional axis coordinate value, T is the reserved parameter please fill in 0
speed   : The speed parameter
outIndex: Trigger the output signal when the robot motion is finished
return  : If success, the return value is 0
**/
LHAPI int   MOVP(float J1, float J2, float J3, float J4, float J5, float J6, float R, float S,
	float T, SPEED speed, unsigned int outIndex);

/**
Offset the specified distance from the current position by joint motion in the world coordinate system
offset  : Offset distance
user    : The index of the user coordinate
tool    : The index of the tool coordinate
speed   : The speed parameter
outIndex: Trigger the output signal when the robot motion is finished
return  : If success, the return value is 0
**/
LHAPI int   BGO(POSITION_OFFSET offset, unsigned int userIndex, unsigned int toolIndex, SPEED speed, unsigned int outIndex);

/**
Offset the specified distance from the current position by linear motion in world coordinate system
offset  : Offset distance
user    : The index of the user coordinate
tool    : The index of the tool coordinate
speed   : The speed parameter
outIndex: Trigger the output signal when the robot motion is finished
return  : If success, the return value is 0
**/
LHAPI int   BMOVE(POSITION_OFFSET offset,  unsigned int userIndex, unsigned int toolIndex, SPEED speed, unsigned int outIndex);

/**
Start or resume the motion function that has been sent 
For example:
MOVJ("P1", 1, speed, 1);
MOVJ("P2", 1, speed, 1);
MOVJ("P3", 1, speed, 1);
APIRun();// Begin to execute motion;
return value: If success, the return value is 0
**/
LHAPI int   APIRun();

/**
Stop the motion process that has been started
return value: If success, the return value is 0
**/
LHAPI int   APIStop();

/**
Pause the motion process that has been started
return value: If success, the return value is 0
**/
LHAPI int   APIPause();

/**
Set the trigger mode of the API motion functions;
triggerMode: The enumeration variable of trigger mode; 0: no trigger mode  1: software trigger mode  2: hardware trigger mode
return     : If success, the return value is 0
**/
LHAPI int   setTriggerMode(TRIGGER_MODE triggerMode);

/**
Get the current API trigger mode
triggerMode: The enumeration variable of trigger mode; 0: no trigger mode  1: software trigger mode  2: hardware trigger mode
return     : If success, the return value is 0
**/
LHAPI int   getTriggerMode(TRIGGER_MODE *triggerMode);

/**
Obtain point details
ptName      : The name of the point
pt          : Pointer to a LPOSITION structure that receives the point information
return value: If success, the return value is 0
**/
LHAPI int   getPoint(const char *ptName, LPOSITION *pt);

/**
Add the current position of the robot to a new point variable
ptName       : The name of the point
ptDescription: Point description
return value : If success, the return value is 0
**/
LHAPI int   teachAddPT(const char *ptName, const char *ptDescription);

/**
Modify point position information
ptName       : The name of the point
ptDescription: Point description
return value : If success, the return value is 0
**/
LHAPI int   teachModifyPT(const char *ptName, const char *ptDescription);

/**
Delete point variable from the points.pt file
ptName       : The name of the point
return value : If success, the return value is 0
**/
LHAPI int   teachDeletePT(const char *ptName);

/**
Set the mode of remote network control;
apiCmdMode   : The enumeration variable of Remote Network mode; 0: DEFAULT mode  1: INTERPRETER mdoe   2: API mode
return value : If success, the return value is 0
**/
//LHAPI int   setApiCmdMode(API_CMD_MODE apiCmdMode);//去掉

/**
Get the current mode of remote network control
apicmdMode   : Pointer to a APICMD_MODE variable that contains the current command mode, 0: DEFAULT mode  1: INTERPRETER mode   2: API mode
return value : If success, the return value is 0
**/
//LHAPI int   getApiCmdMode(API_CMD_MODE *apiCmdMode);//去掉

/**
Enter TEACH mode
return value: If success, the return value is 0
**/
LHAPI int 	beginTeach();

/**
Exit TEACH mode
return value: If success, the return value is 0
**/
LHAPI int 	endTeach();

/**
Select the user coordinate system used in TEACH mode
toolIndex   : The index of the user coordinate system, the value range is 0-16
return value: If success, the return value is 0
**/
LHAPI int 	teachSelectUSER(unsigned int userIndex);

/**
Select the tool coordinate system used in TEACH mode
toolIndex   : The  index of the tool coordinate system, the value range is 0-16
return value: If success, the return value is 0
**/
LHAPI int 	teachSelectTOOL(unsigned int toolIndex);

/**
Begin step move in TEACH mode
stepDistance     : Step move distance, for exmple : 5 or -10.2
speed			 : Step move speed, the range is 1~100
worldName        : The direction of step move, for exmple:"X+" or "Y-" 
stepMode		 : The enumeration variable for the Teach movement mode
return value     : If success, the return value is 0
!!!!!! teachStepMoveBegin() must use with teachStepMoveStop !!!!!!!
**/
LHAPI int 	teachStepMoveBegin(float stepDistance, float speed, char *worldName, STEPMODE stepMode);

/**
Stop the step movement that alredy started in TEACH mode
return value     : If success, the return value is 0
!!!!!! teachStepMoveStop() can not use only !!!!!!!
**/
LHAPI int 	teachStepMoveStop();

/**
Begin continuous movement in TEACH mode
speed            : Jog move speed, the range is 1~100
JointName        : The axis and direction of Jog move, exmple:"X+" or "Y-"
stepMode		 : The enumeration variable for the Teach movement mode
return value     : If success, the return value is 0
!!!!!! teachJogMoveBegin() must use with teachJogMoveStop !!!!!!!
**/
LHAPI int 	teachJogMoveBegin(float speed, char *jointName, STEPMODE stepMode);

/**
Stop the continuous movement that already started in TEACH mode
JointName     : The axis and direction of Jog move, exmple:"X+" or "Y-"
return value  : If success, the return value is 0
!!!!!! teachJogMoveStop() can not use only !!!!!!!
**/
LHAPI int 	teachJogMoveStop(char *jointName);

/**
Set up the working area or the forbidden area of the robot
boxIndex      : Area index, the range is 0-15
boxRange      : The structure that contains BOX range parameters
outputIndex   : Set the output signal of the working area, the range is 0~15
boxOutputMode : Polarity of the area;
INSIDE_ON     : Open the output port after entering the area;
INSIDE_OFF    : Open the output port after leaving the area;
return value  : If success, the return value is 0
**/
LHAPI int   setBOX(unsigned int boxIndex, BOXRANGE boxRange, int outputIndex, OUTPUTMODE boxOutputMode);

/**
Check whether the robot is currently in the specified area
boxIndex    : Area index, the range is 0-15
isInsideBOX : 1-in the box, 0-out of the box
return value: If success, the return value is 0
**/
LHAPI int   insideBox(unsigned int boxIndex, unsigned int *isInsideBOX);


/**
Get the current setting of the specified area
boxIndex    : Area index, the range is 0-15
boxInfo     : The structure containing BOX setting parameters
return value: If success, the return value is 0
**/
LHAPI int   getBoxInfo(unsigned int boxIndex, BOXINFO *boxInfo);

/**
Clear the BOX setting of the robot
boxIndex    : Area index, the range is 0-15
return value: If success, the return value is 0
**/
LHAPI int   clearBOX(unsigned int boxIndex);

/**
Enable or disable the area decision
boxIndex    : Area index, the range is 0-15
enFlag		: 0:unable, 1:enable
return value: If success, the return value is 0
**/
LHAPI int   enableBOX(unsigned int boxIndex, unsigned int enFlag);

/**
Get the current cartesian position information of the robot
pt          : Pointer to a LPOSITION structure that receives the point information
toolIndex   : Tool coordinate system index, the range is 0-16
userIndex   : User coordinate system index, the range is 0-16
return value: If success, the return value is 0
**/
LHAPI int   getRobotCCPOS(LPOSITION *pt, unsigned int toolIndex, unsigned int userIndex);

/**
Get the current joint position information of the robot
Jpt         : Pointer to a JPOSITION structure that receives the joint position information
return value: If success, the return value is 0
**/
LHAPI int   getRobotCJPOS(JPOSITION *Jpt);

/**
Get the last output information from the controller
lastError   : Return error information
iSize       : The size of given string-array “lastError”
return value: If success, the return value is 0
**/
LHAPI int   getLastErrorMsg(char *lastError, unsigned int iSize);

/**
Set the output state according to the output signal index
outputIndex   : Output signal index
return value  : If success, the return value is 0
**/
LHAPI int   setDo(unsigned int outputIndex, unsigned int value);

/**
Get the current output state according to the output signal index
outputIndex   : Output signal index
value         : The output state, 1: on, 0: off
return value  : If success, the return value is 0
**/
LHAPI int   getDo(unsigned int outputIndex, int *value);

/**
Get the current input state according to the input signal index
inputIndex    : Input signal index
value         : The input state, 1: on, 0: off
return value  : If success, the return value is 0
**/
LHAPI int   getDi(unsigned int inputIndex, int *value);

/**
Set the running signal of the API hardware trigger mode
runInputIndex  : The input port that triggers the motion to run, the range is 0~15
return value   : If success, the return value is 0
**/
LHAPI int   setRunSignal(unsigned int runInputIndex);

/**
Set the stop signal of the API hardware trigger mode
stopInputIndex  : The input port that triggers the motion to stop, the range is 0~15
return value    : If success, the return value is 0
**/
LHAPI int   setStopSignal(unsigned int stopInputIndex);

/**
Set the pause signal of the API hardware trigger mode
pauseInputIndex : The input port that triggers the motion to pause, the range is 0~15
return value    : If success, the return value is 0
**/
LHAPI int   setPauseSignal(unsigned int pauseInputIndex);

/**
Get the current setting of the run signal
runInputIndex  : The current setting of the running signal
return value   : If success, the return value is 0
**/
LHAPI int   getRunSignal(int *runInputIndex);

/**
Get the current setting of the stop signal
stopInputIndex  : The current setting of the stop signal
return value    : If success, the return value is 0
**/
LHAPI int   getStopSignal(int *stopInputIndex);

/**
Get the current setting of the pause signal
pauseInputIndex: The current setting of the pause signal
return value   : If success, the return value is 0
**/
LHAPI int   getPauseSignal(int *pauseInputIndex);

///**
//Get the current [maximum cartesian position deceleration]
//fDeceleration : Maximum cartesian position deceleration
//return value  : If success, the return value is 0
//**/
//LHAPI int   getCartesianDecel(float *fDeceleration);

/**
Get the current [maximum cartesian position acceleration]
fAcceleration : Maximum cartesian position acceleration
return value  : If success, the return value is 0
**/
LHAPI int   getCartesianAccel(float *fAcceleration);

/**
Get the current [maximum cartesian position speed]
fSpeed        : Maximum cartesian position speed
return value  : If success, the return value is 0
**/
LHAPI int   getCartesianSpeed(float *fSpeed);//5000


///**
//Get the current [maximum cartesian posture deceleration]
//fPosDeceleration : Maximum cartesian posture deceleration
//return value     : If success, the return value is 0
//**/
//LHAPI int   getCartesianPosDecel(float *fPosDeceleration);

/**
Get the current [maximum cartesian posture acceleration]
fPosAcceleration : Maximum cartesian posture acceleration
return value     : If success, the return value is 0
**/
LHAPI int   getCartesianPosAccel(float *fPosAcceleration);


/**
Get the current [maximum cartesian posture acceleration]
fPosSpeed     : Maximum cartesian posture acceleration
return value  : If success, the return value is 0
**/
LHAPI int   getCartesianPosSpeed(float *fPosSpeed);


/**
Modify the IP and the port
IP            : The IP address of the controller
PORT          : The port which Robot Studio used to connect the controller
return value  : If success, the return value is 0
**/
LHAPI int   setNetworkAddr(char *IP, char *PORT);

/**
Get the DH parameter of the robot body
DHp           : The structure is used to store the DH parameter
return value  : If success, the return value is 0
**/
LHAPI int   getDHparameter(DH *DHp);

/**
Get the current system version 
strVersion    : Return the system version
iSize         : The size of strVersion
return value  : If success, the return value is 0
**/
LHAPI int   getSystemVersion(char *strVersion, unsigned int iSize);

/**
Get the robot body version
strRobotBodyVersion   : Return the current setting of robot body version
iSize                 : The size of given string-array “strRobotBodyVersion”
return value          : If success, the return value is 0
**/
LHAPI int   getRobotBodyVersion(char *strRobotBodyVersion, unsigned int iSize);

/**
Set robot body version
strRobotBodyVersion   : The version of robot to be set
return value          : If success, the return value is 0
**/
LHAPI int   setRobotBodyVersion(char *strRobotBodyVersion);

/**
Get the control cabinet version
strControlCabinetVersion   : Return the current setting of control cabinet version
iSize                      : The size of strControlCabinetVersion
return value               : If success, the return value is 0
**/
LHAPI int   getCtrCabVersion(char *strControlCabinetVersion, unsigned int iSize);

/**
Set the control cabinet version
strControlCabinetVersion   : The version of the control cabinet to be set
return value               : If success, the return value is 0
**/
LHAPI int   setCtrCabVersion(char *strControlCabinetVersion);

/**
Set the origin of the robot
Jindex         : The Joint index, the range is 0~5, represent Joint 1~6
return value   : If success, the return value is 0
**/
LHAPI int   setRobotOrigin(unsigned int Jindex);

/**
Get the current API DLL version
strDllVersion : Return the API version
iSize         : The size of strDllVersion
return value  : If success, the return value is 0
**/
LHAPI int   getApiDllVersion(char *strDllVersion, unsigned int iSize);

/**
Set joints movement range of the robot
fJointRange  : The structure that contains joints movement range parameters
return value : If success, the return value is 0
**/
LHAPI int setJointRange(JOINTRANGE  fJointRange);

/**
Get joints movement range of the robot
fJointRange  : The structure that contains joints movement range parameters
return value : If success, the return value is 0
**/
LHAPI int getJointRange(JOINTRANGE *fJointRange);

/**
Set robot control permissions
iCtrMode     : The structure that contains control parameters
return value : If success, the return value is 0
**/
LHAPI int setRemoteCtrlMode(RemoteControlMode  iRemoteCtrMode);

/**
Get the control mode of the current setting
iCtrMode     : The enumeration variable is used to store the control mode
return value : If success, the return value is 0
**/
LHAPI int getRemoteCtrlMode(RemoteControlMode  *iRemoteCtrMode);


/**
Get the control mode of the current setting
iCtrMode     : The enumeration variable is used to store the control mode
return value : If success, the return value is 0
**/
LHAPI int getCtrlMode(SysControlMode  *iCtrMode);



/**
Set the controller system time
systemTime   : The structure is used to set the system time
return value : If success, the return value is 0
**/
LHAPI int setSysTime(LHSYSTEMTIME systemTime);

/**
Get the controller system time
systemTime   : The structure is used to store the system time
return value : If success, the return value is 0
**/
LHAPI int getSysTime(LHSYSTEMTIME *systemTime);

/**
Set the global speed ratio
iVelPercent  : Global speed percentage, the range is 1~100
return value : If success, the return value is 0
**/
LHAPI int setVelPct(unsigned int iVelPercent);

/**
Get the current global speed ratio
iVelPercent  : Global speed percentage, the range is 1~100
return value : If success, the return value is 0
**/
LHAPI int getVelPct(unsigned int* iVelPercent);

/**
Clear robot battery fault
return value : If success, the return value is 0
**/
LHAPI int clearBatteryFault();

/**
Update the controller system
sysFilePath  : The path of the system update file, for example: "C:\\UpdatePath"
fileName     : The name of the system update file, for example: "sys.bin"
return value : If success, the return value is 0
**/
LHAPI int updateSystem(const char *sysFilePath, char *fileName);

/**
Update the EtherCAT bus configuration file
sysFilePath  : The path of the configuration file, for example: "C:\\UpdatePath"
fileName     : The name of the configuration file, for example: "IO.xml"
return value : If success, the return value is 0
**/
LHAPI int updateXmlFile(const char *xmlFilePath, char *fileName);

/**
Wait for the completion of motion instruction
milliseconds : Watchdog time, stop waiting when time is over
return value : If success, the return value is 0
**/
LHAPI int waitCommandComplete(int milliseconds);

/**
API initialization
return value : If success, the return value is 0
**/
LHAPI int APIinit();

/**
Enable hardware signal to trigger motion functions
return value : If success, the return value is 0
**/
LHAPI int enableIOTrigger();

/**
Convert the joint coordinate system to cartesian coordinate system
return value : If success, the return value is 0
**/
LHAPI int convertJposToCpos(JPOSITION Jpos, LPOSITION *CposOut, unsigned int toolIndex, unsigned int userIndex);
LHAPI int convertJposToCposC(JPOSITION Jpos, LPOSITION *CposOut, CAR_POS Tool, CAR_POS User);
LHAPI int monitorJpos(JPOSITION Jpos);//lfs-2021-1207
/**
Convert the joint coordinate system to cartesian coordinate system
return value : If success, the return value is 0
**/

LHAPI int turnReset(unsigned int jointIndex);

LHAPI int setBodyBox(BODY_BOX bodyBox);

LHAPI int getBodyBox(BODY_BOX *bodyBox);

LHAPI int setToolRadius(float toolRadius);

LHAPI int getToolRadius(float *toolRadius);

LHAPI int getRobotWarningCode(char *warningcode, unsigned int iSize);

LHAPI int getCarMotionParameter(CAR_MOTION_PARAMETER *fCarMotionParameter);

LHAPI int setCarMotionParameter(CAR_MOTION_PARAMETER fCarMotionParameter);

LHAPI int getToolBOX(unsigned int toolBoxIndex, TOOL_BOX_INFO *toolBoxInfo);

LHAPI int setToolBOX(unsigned int toolBoxIndex, TOOL_BOX_INFO toolBoxInfo);

LHAPI int clearToolBOX(unsigned int toolBoxIndex); 

LHAPI int enableToolBOX(unsigned int toolBoxIndex,unsigned int enable);


/**
Set the joint motion parameters of the robot
iJntMotionParameter : The structure is used to store the joint motion parameters
return value        : If success, the return value is 0
**/
LHAPI int   setJntMotionParameter(JOINT_MOTION_PARAMETER iJntMotionParameter);

/**
Get the joint motion parameters of the robot
iJntMotionParameter : The structure is used to store the joint motion parameters
return value        : If success, the return value is 0
**/
LHAPI int   getJntMotionParameter(JOINT_MOTION_PARAMETER *iJntMotionParameter);

/**
Set  arrival judge method of the robot
nArrivalJudge : Arrival judge, 0-Command 1-pulse
return value  : If success, the return value is 0
**/
LHAPI int   setArrivalJudge(unsigned int nArrivalJudge);

/**
Get  arrival judge method of the robot
*nArrivalJudge : Arrival judge, 0-Command 1-pulse
return value  : If success, the return value is 0
**/
LHAPI int   getArrivalJudge(unsigned int *nArrivalJudge);

/**
Get  motion status of the robot
*nMoterStatus : motion status, 0-motion is running 1-motion in stop
return value  : If success, the return value is 0
**/
LHAPI int   getMotorStatus(unsigned int *nMoterStatus);


/**
Set  Reduction ratio
fReductionRatio : Reduction Ratio of anual joint
return value    : If success, the return value is 0
**/
LHAPI int   setReductionRatio(REDUCTION_RATIO fReductionRatio);

/**
Get  Reduction ratio
*fReductionRatio: Reduction ratio of anual joint
return value    : If success, the return value is 0
**/
LHAPI int  getReductionRatio(REDUCTION_RATIO *fReductionRatio);

LHAPI int MOVJ3P(LPOSITION point[2],  unsigned int toolIndex, SPEED speed, unsigned int outIndex);

LHAPI int MOVJ4P(LPOSITION point[3], unsigned int toolIndex, SPEED speed[3], unsigned int outIndex);

LHAPI int MOVCP(LPOSITION point[2], unsigned int toolIndex, SPEED speed, unsigned int outIndex);

LHAPI int MOVC(const char *ptBlend, const char *ptEnd, unsigned int toolIndex, SPEED speed, unsigned int outIndex);

LHAPI int JUMPJ(const char *ptTarget,  unsigned int toolIndex, SPEED upSpeed, SPEED jSpeed, SPEED downSpeed, float upZ, float limitZ, float downZ, unsigned int outIndex);

LHAPI int JUMPL(const char *ptTarget, unsigned int toolIndex, SPEED upSpeed, SPEED lSpeed, SPEED downSpeed, float upZ, float limitZ, float downZ, unsigned int outIndex);

LHAPI int TGO(POSITION_OFFSET ptOffset, unsigned int userIndex, unsigned int toolIndex, SPEED jSpeed, unsigned int outIndex);

LHAPI int TMOVE(POSITION_OFFSET ptOffset, unsigned int userIndex, unsigned int toolIndex, SPEED lSpeed, unsigned int outIndex);

LHAPI int MOVJ_IO(const char *pointName, unsigned int toolIndex, SPEED speed, PROCESS_OUTPUT*processOut,int processOutCount);
/*重载运动函数 MOVL JUMP等*/
LHAPI int MOVL_IO(const char *pointName, unsigned int toolIndex, SPEED speed, PROCESS_OUTPUT*processOut, int processOutCount);

LHAPI int JUMP_IO(const char *pointName, float zLimit, unsigned int toolIndex, SPEED speed, PROCESS_OUTPUT*processOut, int processOutCount);

LHAPI int MOVJP_IO(LPOSITION point, unsigned int toolIndex, SPEED speed, PROCESS_OUTPUT*processOut, int processOutCount);

LHAPI int MOVLP_IO(LPOSITION point, unsigned int toolIndex, SPEED speed, PROCESS_OUTPUT*processOut, int processOutCount);

LHAPI int JUMPP_IO(LPOSITION point, float zLimit, unsigned int toolIndex, SPEED speed, PROCESS_OUTPUT*processOut, int processOutCount);

LHAPI int MOVP_IO(LHJPOSITION jPos, SPEED speed, PROCESS_OUTPUT*processOut, int processOutCount);

LHAPI int MOVC_IO(const char *ptBlend, const char *ptEnd, unsigned int toolIndex, SPEED speed, PROCESS_OUTPUT*processOut, int processOutCount);

LHAPI int JUMPJ_IO(const char *ptTarget, unsigned int toolIndex, SPEED upSpeed, SPEED jSpeed, SPEED downSpeed, float upZ, float limitZ, float downZ, PROCESS_OUTPUT*processOut, int processOutCount);

LHAPI int JUMPL_IO(const char *ptTarget, unsigned int toolIndex, SPEED upSpeed, SPEED lSpeed, SPEED downSpeed, float upZ, float limitZ, float downZ, PROCESS_OUTPUT*processOut, int processOutCount);

LHAPI int MOVCP_IO(LPOSITION point[2], unsigned int toolIndex, SPEED speed, PROCESS_OUTPUT*processOut, int processOutCount);

LHAPI int setDoWB(unsigned int outputIndex, unsigned int value);

LHAPI int bindGIN(unsigned int groupID, unsigned int *inputIndexArray,int arraySize);/*inputIndexArray 数组支持8个最大*/

LHAPI int bindGOUT(unsigned int groupID, unsigned int *outputIndexArray,int arraySize);/*outputIndexArray 数组支持8个最大*/

LHAPI int clearGIN(unsigned int groupID);

LHAPI int clearGOUT(unsigned int groupID);

//LHAPI int setGIN(unsigned int groupID,unsigned int value);
///*写入 10 进制时，范围为 0～255*/

LHAPI int setGOUT(unsigned int groupID, unsigned int value);

LHAPI int getGOUT(unsigned int groupID, unsigned int *value);

LHAPI int getGIN(unsigned int groupID,unsigned int *value);

LHAPI int setPallet(unsigned int palletID, const char *point1, const char *point2, const char *point3, unsigned int splitRow, unsigned int splitCol);

LHAPI int clearPallet(unsigned int palletID);

LHAPI int pallet(unsigned int palletID, unsigned int ptIndex, LPOSITION *lPoint);

LHAPI int waitIN(unsigned int inputIndex, int milliseconds);

LHAPI int setDHparameter(DH DHp);
LHAPI int getTool(unsigned int toolBoxIndex, CAR_POS *tool);//lfs-2021-0726
LHAPI int taskRun();

LHAPI int taskStop();

LHAPI int taskPause();

LHAPI int taskResume();

LHAPI int getTaskStatus(TaskStatus *taskStatus);

LHAPI int clearTaskFault();

typedef enum
{
	err_ERROR = 0x00,   /*停止状态*/
	err_WARNING = 0x01,
	err_EVENT = 0x02,   /*停止状态*/
} ErrType;

typedef enum
{
	lang_CHS = 0x00,   /*停止状态*/
	lang_ENG = 0x01,
} LangType;

LHAPI void getErrorMsg(char*filePath,int errCode, ErrType errType, LangType langType, char*errMsg);

LHAPI int getIsSystemIO(int IoIndex,int *isSystem);

LHAPI int   getPointCount(int *ptCount);

LHAPI int   getPointNameAndDes(int ptIndex,char *ptName, char *ptDes);

LHAPI int   getInputCount(int *inputCount);
LHAPI int   getOutputCount(int *outputCount);
LHAPI int   setTeachActMOVJ(const char *pointName, unsigned int toolIndex, int speed);
LHAPI int   setTeachActMOVL(const char *pointName, unsigned int toolIndex, int speed);
LHAPI int   setTeachActMOVC(const char *pointName1, const char *pointName2, unsigned int toolIndex, int speed);
LHAPI int   setTeachActJUMP(const char *pointName, float offset, float Zlimit, unsigned int toolIndex,int speed);

//lfs adds for test-2021-1217
/****************************************************
设置远程控制模式
****************************************************/
LHAPI int setExternalMode(int ExternalMode);
/****************************************************
设置远程控制模式
****************************************************/
LHAPI int getRobotType(int  *i_RobotType);
/****************************************************
把当前点设为零点
****************************************************/
LHAPI int set_Home();
/****************************************************
设定虚拟IO
****************************************************/
LHAPI int setSimuIO(int SimuIO);

LHAPI int setElecValve(int index, OUTPUTMODE value);

typedef enum _FuctionId {
	Serial = 0,  	/*串口  */
	ModbusRtu = 1 , /*Modbus_rtu */
	ModbusTcp = 2   /*Modbus_tcp */
}FuctionId;

typedef enum _BaudRate {
	Rate2400 = 0,	
	Rate4800 = 1,  
	Rate9600 = 2,
	Rate14400 = 3,
	Rate19200 = 4,
	Rate38400 = 5,
	Rate57600 = 6,
	Rate115200 = 7,
}BaudRate;

typedef enum _Parity {
	NoParity = 0,
	OddParity = 1,
	EvenParity = 2,
}Parity;

typedef enum _StopBit {
	One = 0, //1
	OnePointFive = 1, //1.5
	Two = 2, //2
}StopBit;


typedef struct _ModbusPara
{
	int nComNameId; //串口ID 支持 0,1,2
	FuctionId nFuncTypeId;//0-串口  ,1-MODBUS_RTU,2-MODBUS_TCP
	BaudRate nBaudRate; //波特率 2400 4800 9600 14400 19200 38400 57600 115200 
	int nByteSize; //0--4,1--5,2--6,3--7,4--8
	Parity nParityId; //校验位 0-无校验 1-奇校验 2-偶校验
	StopBit nStopBit; // 0-- 1  1---1.5 2---2 
}ModbusPara;

LHAPI int setModbusPara(ModbusPara MBvalue);

typedef struct _GlueSystemTick
{
	double systemTick[100];
}GlueSystemTick;

LHAPI int getGlueSystemTick (int glueIndex, int  *gSysTick,int arraySize);

LHAPI int exportGlueProject(const char *prjPath, const char *fileName);

LHAPI int importGlueProject(const char *prjPath, const char *fileName);

LHAPI int modifyPointByValue(char *ptOldName, char *ptNewName, char *ptDes, LPOSITION pointValue);


/*
开始采集
*/
LHAPI int startSample();

/*
停止采集
*/
LHAPI int stopSample();


/*
获取电机电流
*/
LHAPI int getMotorCurrent(char * buf,unsigned int bufsize);
/*
获取电机速度
*/
LHAPI int getMotorSpeed(char * buf, unsigned int bufsize);
/*
获取电机位置
*/
LHAPI int getMotorPos(char * buf, unsigned int bufsize);
/*
获取机器人笛卡尔位置
*/
LHAPI int getCarPos(char * buf, unsigned int bufsize);
/*
获取机器人关节位置
*/
LHAPI int getJntPos(char * buf, unsigned int bufsize);
/*
设置频率
*/
LHAPI int setFrequency(unsigned int frequency);

/*
获取频率
*/
LHAPI int getFrequency(unsigned int *frequency);

/**
Get sdo value
axisNum : The range is 1-6
sdoIndex :The index of sdo
sdoSubIndex:The sub index of sdo
bufsdoMsg:display sdo message
bufSize:sdo message array size
return value  : If success, the return value is 0
**/
LHAPI int getSdo(unsigned int axisNum, unsigned int sdoIndex, unsigned int sdoSubIndex, char *bufsdoMsg, int bufSize);

/**
Set sdo value
axisNum : The range is 1-6
sdoIndex :The index of sdo
sdoSubIndex:The sub index of sdo
sdoValue: set sdo velue
return value  : If success, the return value is 0
**/
LHAPI int setSdo(unsigned int axisNum, unsigned int sdoIndex, unsigned int sdoSubIndex, int sdoValue);

LHAPI int setTeachActMOVJP(LPOSITION ptMove, unsigned int toolIndex, unsigned int userIndex, int speed);
LHAPI int setTeachActMOVLP(LPOSITION ptMove, unsigned int toolIndex, unsigned int userIndex, int speed);
LHAPI int setTeachActMOVCP(LPOSITION ptMove1, LPOSITION ptMove2, unsigned int toolIndex, unsigned int userIndex, int speed);
LHAPI int setTeachActJUMPP(LPOSITION ptMove, float offset, float Zlimit, unsigned int toolIndex, unsigned int userIndex, int speed);

#endif