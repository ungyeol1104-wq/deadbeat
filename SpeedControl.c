
#include "GlobalVar.h"
#include "math.h"
#include "MotorControl.h"
#include "UserMath.h"

/**
 * @brief  LADRC 속도 제어기 파라미터 및 변수들을 초기화
*/
void vInitSpeedControl(sMotorCtrl* MotorControl, sSpeedCtrl* SCtrl){
    /* 1. 지령 및 램프 초기화 */
    SCtrl->fWrpmRefSet = 0.0f;
    SCtrl->fWrpmRef = 0.0f;

    /* 2. LADRC 제어 파라미터 설정 (초기 튜닝값) */
     SCtrl->fWc = 10.0f;   // 속도 추종 대역폭 (Control Bandwidth)
     SCtrl->fWo = 15.0f;  // 외란 관측 대역폭 (Observer Bandwidth, 보통 3~5 * Wc)
     SCtrl->fB0 = 2000.0f;   // 시스템 게인 추정치 (관성에 비례, 적절한 값에서 시작)

    /* 3. LESO 상태 변수 초기화 */
     SCtrl->fZ1 = 0.0f;     // 추정 속도 (RPM)
     SCtrl->fZ2 = 0.0f;     // 추정 총 외란 (Disturbance)
     SCtrl->fU_Prev = 0.0f; // 이전 스텝의 Iq 지령

    /* 4. 출력 제한 설정 (모터 정격 전류 기반) */
     SCtrl->fTeRefMax = MotorControl->IS_RATED;
     SCtrl->fTeRefMin = -MotorControl->IS_RATED;

     SCtrl->fIqsrRefSC = 0.0f;
}

/**
* @brief  LADRC 기반 속도 제어 알고리즘을 수행합니다.
  */
 void vSpeedControl(sMotorCtrl* MotorControl, sSpeedObs* SObs, sSpeedCtrl* SCtrl){

     /* 1. 속도 지령 프로파일 생성 (Ramp 적용) */
     if (SCtrl->fWrpmRefSet >= SCtrl->fWrpmRef) {
         vSlopeGenerator(&SCtrl->fWrpmRef, SCtrl->fWrpmRefSet, 1000.0f * fTSc);
     } else {
         vSlopeGenerator(&SCtrl->fWrpmRef, SCtrl->fWrpmRefSet, 0.5f * 1000.0f * fTSc);
     }

     /* 2. LADRC - LESO (상태 및 외란 관측기) */
     // fTSc: 속도 제어 주기 (약 2ms)
     float fBeta1 = 2.0f * SCtrl->fWo;
     float fBeta2 = SCtrl->fWo * SCtrl->fWo;

     // 관측기 오차 (추정 속도 - 실제 측정 속도)
     float fError_Obs = SCtrl->fZ1 - SObs->fWrpmSC;

     // 오일러 적분 기반 상태 업데이트
     SCtrl->fZ1 += fTSc * (SCtrl->fZ2 + SCtrl->fB0 * SCtrl->fU_Prev - fBeta1 * fError_Obs);
     SCtrl->fZ2 += fTSc * (-fBeta2 * fError_Obs);

     /* 3. LADRC - LSEF (제어 법칙 및 외란 상쇄) */
     // u0 = Wc * (지령속도 - 추정속도)
     float fU0 = SCtrl->fWc * (SCtrl->fWrpmRef - SCtrl->fZ1);

     // 최종 출력 계산: (u0 - 추정외란) / B0
     float fIq_Target = (fU0 - SCtrl->fZ2) / SCtrl->fB0;

     /* 4. 출력 제한 (Saturation) */
     SCtrl->fIqsrRefSC = LIMIT(fIq_Target, SCtrl->fTeRefMin, SCtrl->fTeRefMax);

     /* 5. 이전 출력값 저장 (다음 스텝 관측기용) */
     SCtrl->fU_Prev = SCtrl->fIqsrRefSC;
 }
