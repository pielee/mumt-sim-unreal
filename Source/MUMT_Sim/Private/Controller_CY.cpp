#include "Controller_CY.h"
#include <cmath>

// 원본은 전역 clamp — 유니티 빌드/타 모듈과의 심볼 충돌을 피하려고 static만 부여 (로직 동일).
static float clamp(float input, float RangeDown, float RangeUp)
{
	if (input <= RangeDown)
	{
		return RangeDown;
	}
	else if (input >= RangeUp)
	{
		return RangeUp;
	}
	else
	{
		return input;
	}
}

StickController::StickController()
{
	SumCount = 0;
	for (int i = 0; i < 20; i++)
		MF[i] = 0;
	FilterIndex = 0;

	for (int i = 0; i < 60; i++)
		ErrorSum.push_back(0.0);
}

float StickController::GetLOSErrorSUM(float LOSError)
{
	SumCount++;
	if (LOSError <= 10)
	{
		if (SumCount < 60)
			ErrorSum.push_back(LOSError);
		else
			ErrorSum[SumCount % 60] = LOSError;
	}
	else
	{
		if (ErrorSum.size() < 60)
			ErrorSum.push_back(0);
		else
			ErrorSum[SumCount % 60] = 0;
	}
	int sum = 0;   // 원본 유지: int 누적(절단)도 원본 동작 그대로

	for (int i = 0; i < ErrorSum.size(); i++)
	{
		sum += ErrorSum[i];
	}

	float Re = sum / 60;

	return Re;
}

StickValue StickController::GetStick(Vector3 MyLocation_FNED, Vector3 MyRotation_FNED, Vector3 VP)
{
	Vector3 Mylocation(MyLocation_FNED.X, MyLocation_FNED.Y, MyLocation_FNED.Z);
	Vector3 TargetLocation(VP.X, VP.Y, VP.Z);

	// 오일러 각을 입력. 이 부분은 언리얼4의 각도를 회사의 ECEF_LLA_Converter 쪽의 각도와 함수들을 이용하기 위해 이쪽 양식에 맞추는 과정
	EulerAngle EA;
	EA.Roll = MyRotation_FNED.X;
	EA.Pitch = MyRotation_FNED.Y;
	EA.Yaw = MyRotation_FNED.Z;

	// 오일러각을 이용하면 축변환에 따른 오차가 생기기 때문에 쿼터니언으로 변환하여 사용
	Quaternion QU = EA.toQuaternion();

	// 쿼터니언을 이용하여 전방벡터(ForwardVector)를 생성
	Vector3 ForwardVector;
	ForwardVector.X = 1 - 2 * (QU.X * QU.X + QU.Y * QU.Y);
	ForwardVector.Y = 2 * (QU.X * QU.Z + QU.W * QU.Y);
	ForwardVector.Z = -2 * (QU.Y * QU.Z - QU.W * QU.X);

	// 쿼터니언을 이용하여 수직벡터(UpVector)를 생성
	Vector3 UpVector;
	UpVector.X = -2 * (QU.Y * QU.Z + QU.W * QU.X);
	UpVector.Y = -2 * (QU.X * QU.Y - QU.W * QU.Z);
	UpVector.Z = 1 - 2 * (QU.X * QU.X + QU.Z * QU.Z);

	// 쿼터니언을 이용하여 오른쪽벡터(RightVector)를 생성
	Vector3 RightVector;
	RightVector.X = 2 * (QU.X * QU.Z - QU.W * QU.Y);
	RightVector.Y = 1 - 2 * (QU.Y * QU.Y + QU.Z * QU.Z);
	RightVector.Z = -2 * (QU.X * QU.Y + QU.W * QU.Z);


	Vector3 ForwardVectorPoint = ForwardVector * 1000 + Mylocation;

	Vector3 ForwardVectorPoint2VP = TargetLocation - ForwardVectorPoint;

	Vector3 Proj_V = (ForwardVectorPoint2VP.dot(ForwardVector)) * ForwardVector;

	Vector3 Proj_P = TargetLocation - Proj_V;
	Vector3 Proj_TV = Proj_P - ForwardVectorPoint;

	// 롤커맨드 생성 부분

	float UpVector2Proj_TV_Angle = std::acos(UpVector.dot(Proj_TV / Proj_TV.length()));
	float UTAngle;
	float LOS = std::acos(ForwardVector.dot((TargetLocation - Mylocation)) / (TargetLocation - Mylocation).length()) * RADTODEG;

	if (std::isnan(UpVector2Proj_TV_Angle))
	{
		UpVector2Proj_TV_Angle = 0;
	}

	float Proj_TV_Length = Proj_TV.length();

	if(Proj_TV_Length <= 0)
	{
		Proj_TV_Length = 0.0001;
	}

	if (RightVector.dot(Proj_TV / Proj_TV_Length) >= 0)
	{
		UTAngle = UpVector2Proj_TV_Angle;
	}
	else
	{
		UTAngle = UpVector2Proj_TV_Angle * (-1);
	}

	float RollCMD;

	if (std::abs(UTAngle * RADTODEG) > 90)
	{
		RollCMD = (std::sin(UTAngle) * 1);

		if (LOS > 3)
			RollCMD = clamp(RollCMD, -1, 1);
		else
			RollCMD = RollCMD * LOS * (-0.1);
	}
	else
	{
		RollCMD = (std::sin(UTAngle) * 1.0);

		RollCMD = clamp(RollCMD, -1, 1);

		RollCMD = RollCMD * std::abs(RollCMD);
	}


	if (std::isnan(LOS))
	{
		LOS = 0;
	}

	if (RollCMD < 0.1)
		RollCMD = RollCMD * 3;

	RollCMD = RollCMD * clamp(LOS, 0, 1);
	// 러더 커맨드 생성 부분
	float RudderCMD = 0;

	RudderCMD = -std::sin(UTAngle) * clamp(LOS, 0, 6) * 1;

	MF[FilterIndex % 20] = RudderCMD;
	FilterIndex++;

	int MFsum = 0;   // 원본 유지: int 누적(절단)도 원본 동작 그대로
	for (int i = 0; i < 20; i++)
		MFsum += MF[i];
	RudderCMD = (MFsum / 20 + RudderCMD) / 2;

	// 피치 커맨드 생성 부분
	float PitchCMD = 0;;

	float ERROR_Effect = clamp(LOS / 6 + clamp(GetLOSErrorSUM(LOS) / 7.5, 0, 0.25), 0, 1.5);
	//float ERROR_Effect = clamp(LOS / 6, 0, 1.5);


	float Roll_Effect = 1 - clamp(std::abs(UTAngle * RADTODEG) / 90, 0, 1);

	float Horizon_Effect;
	if (std::abs(UTAngle * RADTODEG) <= 90)
	{
		Horizon_Effect = 1;
	}
	else
		Horizon_Effect = 0.5;

	//std::cout << "ERROR_Effect : " << ERROR_Effect << " Roll_Effect : " << Roll_Effect << " Horizon_Effect : " << Horizon_Effect << std::endl;

	if (LOS < 90)
		PitchCMD = ERROR_Effect * Roll_Effect * Horizon_Effect * (-1);//+Roll_Effect2;
	else
		PitchCMD = -1;

	StickValue Result;
	Result.RollCMD = clamp(RollCMD, -1, 1);
	Result.PitchCMD = clamp(PitchCMD, -1, 1);
	Result.RudderCMD = clamp(RudderCMD, -1, 1);
	//Result.RudderCMD = RudderCMD;
	return Result;
}

// ─── 정점유지(station-keeping) 모드 ──────────────────────────────────────────
// 기하 계산은 GetStick과 동일, 조종 법칙만 편대/웨이포인트용으로 교체.
// 설계 근거·법칙은 헤더 주석 참조. 원본 GetStick은 무수정 보존.
StickValue StickController::GetStickStation(Vector3 MyLocation_FNED, Vector3 MyRotation_FNED, Vector3 VP)
{
	Vector3 Mylocation(MyLocation_FNED.X, MyLocation_FNED.Y, MyLocation_FNED.Z);
	Vector3 TargetLocation(VP.X, VP.Y, VP.Z);

	EulerAngle EA;
	EA.Roll = MyRotation_FNED.X;
	EA.Pitch = MyRotation_FNED.Y;
	EA.Yaw = MyRotation_FNED.Z;
	Quaternion QU = EA.toQuaternion();

	Vector3 ForwardVector;
	ForwardVector.X = 1 - 2 * (QU.X * QU.X + QU.Y * QU.Y);
	ForwardVector.Y = 2 * (QU.X * QU.Z + QU.W * QU.Y);
	ForwardVector.Z = -2 * (QU.Y * QU.Z - QU.W * QU.X);

	Vector3 UpVector;
	UpVector.X = -2 * (QU.Y * QU.Z + QU.W * QU.X);
	UpVector.Y = -2 * (QU.X * QU.Y - QU.W * QU.Z);
	UpVector.Z = 1 - 2 * (QU.X * QU.X + QU.Z * QU.Z);

	Vector3 RightVector;
	RightVector.X = 2 * (QU.X * QU.Z - QU.W * QU.Y);
	RightVector.Y = 1 - 2 * (QU.Y * QU.Y + QU.Z * QU.Z);
	RightVector.Z = -2 * (QU.X * QU.Y + QU.W * QU.Z);

	Vector3 ForwardVectorPoint = ForwardVector * 1000 + Mylocation;
	Vector3 ForwardVectorPoint2VP = TargetLocation - ForwardVectorPoint;
	Vector3 Proj_V = (ForwardVectorPoint2VP.dot(ForwardVector)) * ForwardVector;
	Vector3 Proj_TV = (TargetLocation - Proj_V) - ForwardVectorPoint;

	float UpVector2Proj_TV_Angle = std::acos(UpVector.dot(Proj_TV / Proj_TV.length()));
	float LOS = std::acos(ForwardVector.dot((TargetLocation - Mylocation)) / (TargetLocation - Mylocation).length()) * RADTODEG;

	if (std::isnan(UpVector2Proj_TV_Angle))
	{
		UpVector2Proj_TV_Angle = 0;
	}
	if (std::isnan(LOS))
	{
		LOS = 0;
	}

	float Proj_TV_Length = Proj_TV.length();
	if (Proj_TV_Length <= 0)
	{
		Proj_TV_Length = 0.0001;
	}

	float UTAngle = (RightVector.dot(Proj_TV / Proj_TV_Length) >= 0)
	              ? UpVector2Proj_TV_Angle : -UpVector2Proj_TV_Angle;

	// Roll: 연속 비례 — 부호 반전/×3 부스트 없음, LOS 3° 이내 선형 감쇠
	float RollCMD = clamp(std::sin(UTAngle), -1, 1);
	RollCMD = RollCMD * std::abs(RollCMD);
	RollCMD = RollCMD * clamp(LOS / 3.0f, 0, 1);

	// Pitch: 양방향 비례 — (표적 앙각 − 자세 피치), 6°에서 풀 데플렉션
	Vector3 Rel = TargetLocation - Mylocation;
	float HorizDist = std::sqrt(Rel.X * Rel.X + Rel.Y * Rel.Y);
	float ElevDeg = std::atan2(Rel.Z, HorizDist) * RADTODEG;
	float PitchDeg = MyRotation_FNED.Y * RADTODEG;
	float PitchCMD = -clamp((ElevDeg - PitchDeg) / 6.0f, -1, 1);

	StickValue Result;
	Result.RollCMD = clamp(RollCMD, -1, 1);
	Result.PitchCMD = clamp(PitchCMD, -1, 1);
	Result.RudderCMD = 0.0f;   // 정점유지: 러더 미사용 (에일러론 협조선회)
	return Result;
}
