/*
	JSBSim을 조종하기 위한 스틱 롤링 제어기 (원본: AIP 프로젝트 Controller_CY)

	추적점을 통과하기 위한 조종값을 만드는 제어기가 아닌 추적점을 바라보기 위한 조종값을 만드는 것을 목표로 함

		-> 추적점을 통과하는 제어기는 해당 점으로 이동하는 방법에는 정확할지 모르겠으나 조준 성능이 떨어져 해당 AIP 프로젝트에서는 사용 불가능

	간략한 개요

		- 내 forwardVector를 법선으로 가지는 가상의 평면을 생성
		- 적기를 해당 평면에 프로젝션
		- 내 ForwardVector와 평면이 만나는 점에서 프로젝션된 적기의 위치까지의 벡터를 구함 : V
		- 내 UpVector와 V와의 각도를 통하여 Roll Cmd 생성
		- 해당 각도와 Los 등을 이용하여 pitch cmd 생성
		- Rudder Cmd는 RollCMD와 los 값을 이용하여 생성

		자세한건 코드보고 좀 읽어보시길

	── MUMT_Sim 이식 노트 (2026-07-05) ─────────────────────────────────
	원본 구조/로직 그대로 이식. 원본과 다른 점:
	  - 의존 헤더(Vector3/EulerAngle/Quaternion)는 원본이 제공되지 않아
	    BT_Geometry/ 아래에 동일 인터페이스로 재구성 (규약은 수치 역공학으로
	    검증 — EulerAngle.h 주석 참조). Matrix3/CoordinateConverter는 이
	    파일에서 미사용이라 include 제거.
	  - RADTODEG 상수는 원본에선 외부 헤더 제공으로 추정 → 여기 정의.
	  - _isnan(MSVC 전용) → std::isnan (Linux/clang 호환).
	좌표 규약: 위치 = (North, East, Up[고도 양수]) Cartesian(m),
	           자세 = FNED 오일러 라디안 (JSBSim 출력 그대로).
*/
#pragma once

#include "BT_Geometry/Vector3.h"
#include "BT_Geometry/EulerAngle.h"
#include "BT_Geometry/Quaternion.h"
#include <vector>

using namespace BT_Geometry;

const double DEG2RAD = 3.14159265358979323846 / 180.0;
const double RADTODEG = 180.0 / 3.14159265358979323846;

struct StickValue
{
	float RollCMD;
	float PitchCMD;
	float RudderCMD;
};

class StickController
{
	int SumCount;
	float MF[20];
	int FilterIndex;
	std::vector<float> ErrorSum;

public:
	StickController();
	float GetLOSErrorSUM(float LOSError);
	/*
		일반적인 기동에서 사용하는 제어기

		MyLocation_FNED : Cartesian 	/ 고도를 양수로 가지는 평면 좌표계
		MyRotation_FNED : Radian		/ 평면 좌표계 기준 자세(오일러. JSBSim 출력값 라디안 버전)
		VP				: Cartesian 	/ 이 비행기가 바라봐야할 위치
	*/
	StickValue GetStick(Vector3 MyLocation_FNED, Vector3 MyRotation_FNED, Vector3 VP);

};
