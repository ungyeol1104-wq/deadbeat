/**
 * @file    SpeedControl.h
 * @author  lsj50
 * @date    Sep 4, 2025
 * @brief   전동기 속도 제어기(ASR, Adjustable Speed Regulator) 정의 헤더 파일
 * @details 속도 PI 제어를 위한 구조체와 차단 주파수 상수를 포함하며,
 * 피드백된 속도와 지령 속도 사이의 오차를 바탕으로 토크(q축 전류) 지령을 생성합니다.
 */

#ifndef INC_SPEEDCONTROL_H_
#define INC_SPEEDCONTROL_H_



/**
 * @struct sSpeedCtrl
 * @brief  속도 제어 루프의 상태 변수 및 이득을 관리하는 구조체
 */
typedef struct {
	// 1. 속도 지령 및 램프 (Speed Reference & Ramp)
	float fWrpmRefSet;      /**< 사용자 또는 상위 제어기에서 설정한 목표 RPM */
	float fWrpmRef;         /**< 가속/감속 램프가 적용된 실제 지령 RPM */

    // 2. LADRC 제어 파라미터
	float fWc;              /**< 속도 추종 대역폭 (Control Bandwidth) */
	float fWo;              /**< 외란 관측 대역폭 (Observer Bandwidth) */
	float fB0;              /**< 시스템 게인 추정치 (System Gain) */

	// 3. LESO 상태 변수 (추가)
     float fZ1;              /**< LESO 추정 속도 (RPM) */
     float fZ2;              /**< LESO 추정 총 외란 (Disturbance) */
     float fU_Prev;          /**< 이전 스텝의 출력 전류 지령 (Iq_ref) */

	// 4. 출력 및 제한 (Output & Limits)
	volatile float fIqsrRefSC;       /**< 속도 제어기 출력인 q축 전류 지령값 (토크 성분) */
	float fTeRefMax;        /**< 출력 토크의 최대 제한치 */
	float fTeRefMin;        /**< 출력 토크의 최소 제한치 */

} sSpeedCtrl;



/* === 함수 프로토타입 === */
/* 필요 시 vInitSpeedControl, vSpeedControl 등의 프로토타입을 여기에 추가할 수 있습니다. */
/**
 * @brief  LADRC 속도 제어기 파라미터를 초기화합니다.
 * @param  SCtrl 속도 제어 구조체 포인터
*/
void vInitSpeedADRC(sSpeedCtrl* SCtrl);

#endif /* INC_SPEEDCONTROL_H_ */
