// TailPlc.h

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#ifndef TAILPLC_DLL
#define TAILPLC_DLL __declspec(dllimport)
#endif // #ifndef TAIPLC_DLL

TAILPLC_DLL long ConnectPlc(char* szIp, long nPort, long nAddr); // 连接PLC
TAILPLC_DLL long IsPlcConnected(); // PLC连接状态
TAILPLC_DLL void ClosePlc(); // 断开PLC

TAILPLC_DLL void ClampLock(); // 夹爪夹紧
TAILPLC_DLL void ClampRelease(); // 夹爪张开

// 先拧松再拧紧
TAILPLC_DLL void SetRunReleaseTurns(double dTurns); // 设置拧松圈数
TAILPLC_DLL void SetRunReleaseSpeed(double dSpeed); // 设置拧松速度
TAILPLC_DLL void SetRunLockTurns(double dTurns); // 设置拧紧圈数
TAILPLC_DLL void SetRunLockSpeed(double dSpeed); // 设置拧紧速度
TAILPLC_DLL void RunLock(); // 先拧松再拧紧动作

// 直接拧紧
TAILPLC_DLL void SetDirectReleaseTurns(double dTurns); // 设置拧松圈数
TAILPLC_DLL void SetDirectReleaseSpeed(double dSpeed); // 设置拧松速度
TAILPLC_DLL void DirectRelease(); // 直接拧松

// 直接拧紧
TAILPLC_DLL void SetDirectLockTurns(double dTurns); // 设置拧紧圈数
TAILPLC_DLL void SetDirectLockSpeed(double dSpeed); // 设置拧紧速度
TAILPLC_DLL void DirectLock(); // 直接拧紧

TAILPLC_DLL void Estop(); // 紧急停止
TAILPLC_DLL long IsMotorRunning();  // 判断电动是否在动作
TAILPLC_DLL long IsLimitPos(); // 限位信号
TAILPLC_DLL float ReadMotorCurrent(); // 读取电机电流


TAILPLC_DLL long OpenPort(char* szPort); // 打开串口
TAILPLC_DLL void ClosePort(); // 关闭串口
TAILPLC_DLL void SetBaudRate(long nBaudRate); // 设置波特率
TAILPLC_DLL void WritePortData(short nStartVol, short nEndVol, short nIncVol, short nPeriod, short nDelay); // 写数据
