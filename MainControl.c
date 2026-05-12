/**
 * @file    MainControl.c
 * @author  lsj50
 * @date    Aug 25, 2025
 * @brief   모터 제어 메인 인터럽트 루프 및 상태 머신(State Machine) 관리 소스 파일
 * TIM1 인터럽트에서 50µs(20kHz) 주기로 호출되는 vControl() 함수를 통해
 * FOC(Field Oriented Control) 기반 모터 제어 및 보호 로직을 수행한다.
 *
 * @details [메인 제어 루프 실행 순서]
 * 1. 연산 시간 모니터링을 위한 CPU 사이클 카운트 시작
 * 2. 홀 센서 기반 회전자 위치 및 각도 정보 갱신 (fGetHallSensorInfo)
 * 3. H/W 및 S/W 고장(Fault) 검사: 과전압, 과전류, 과속도 감지 시 즉시 예외 처리
 * 4. 리셋(Reset) 명령 처리 및 시스템 제어기 초기화
 * 5. 상태 머신(State Machine) 및 제어 모드(uControlMode)에 따른 제어 로직 수행
 * 6. 내부 변수 디버깅용 DAC 출력 (vIntDacOut) 및 제어 루프 소요 시간 계산
 *
 * @details [상태 머신 (State Machine) 구조]
 * | 상태 (State) | 주요 동작 및 특징 |
 * | :--- | :--- |
 * | **IDLE** | 제어기 초기화 및 PWM 차단. START 명령 시 부트스트랩 충전 후 상태 전이 대기 |
 * | **ALIGN** | FOC 구동 전 회전자 초기 위치 정렬 수행. 정렬 완료 후 모드에 따라 전이 |
 * | **RUN** | 20kHz 주기로 전류 제어 및 전압 변조(SVPWM) 수행, 분주기(uSpdCnt)를 통한 속도 제어 수행 |
 * | **FAULT** | 시스템 고장 감지 시 PWM을 즉시 차단하고 구동을 중지하여 하드웨어 보호 |
 *
 * @details [제어 모드 (uControlMode)에 따른 동작 분기]
 * 부트스트랩 충전 및 ALIGN 완료 시, `uControlMode` 설정값에 따라 상태 전이 시퀀스가 달라집니다.
 * | 제어 모드 (uControlMode) | 동작 흐름 (State Transition Sequence) | 설명 |
 * | :--- | :--- | :--- |
 * | **DUTY_TEST_MODE**<br>**CONST_VOLT_MODE** | IDLE &rarr; RUN | 위치 정렬(ALIGN)이 필요 없는 테스트/전압 개루프 모드. 바로 RUN 상태로 진입. |
 * | **ALIGN_MODE** | IDLE &rarr; ALIGN &rarr; IDLE | 회전자 위치 정렬만 단독으로 수행하고 다시 대기(IDLE) 상태로 복귀. |
 * | **일반 구동 모드**<br>(FOC 등) | IDLE &rarr; ALIGN &rarr; RUN | 정상적인 모터 구동을 위해 위치 정렬 완료 후 RUN 상태로 진입. |
 */
#include "GlobalVar.h"
#include "UserMath.h"
#include "MotorControl.h"
#include "IntDac.h"

/** @brief 제어 루프 시작 시점의 CPU 사이클 카운트 저장 변수 */
uint32_t ulControlStartClock = 0ul;
/** @brief 제어 루프 연산 소요 시간 (us 단위) */
float fElapsedTimeUs = 0.0f;
/** @brief CAN 통신 송신 횟수 카운터 */
uint16_t uCANTxCnt = 0u;

/** @brief PWM 출력을 담당하는 타이머 1 핸들러 외부 참조 */
extern TIM_HandleTypeDef htim1;

/** @brief 시스템 제어 플래그(시작, 리셋 등)를 담은 레지스터 구조체 */
FLAG_REG Flag;

/** @brief 듀티 테스트용 디버깅 변수 1 */
volatile float fDutyTest1 = 0.5f;
/** @brief 듀티 테스트용 디버깅 변수 2 */
volatile float fDutyTest2 = 0.3f;
/** @brief 듀티 테스트용 디버깅 변수 3 */
volatile float fDutyTest3 = 0.7f;

/** @brief 모터 제어 상태 머신 관리를 위한 정적 변수 (이전, 현재, 다음 상태) */
uint16_t uCurrState = IDLE_STATE;
static uint16_t uPrevState = IDLE_STATE, uNextState = IDLE_STATE;

/** @brief CAN 통신 송신 모드 설정 변수 */
uint16_t uCANTxMode = 0u;

/** @brief 시스템 시작 플래그 (디버깅/테스트용 변수) */
/** @brief 시스템 리셋 플래그 (디버깅/테스트용 변수) */
uint16_t uFlag_Reset = 0u;
/** @brief 저속 제어(Low Speed Control) 주기를 맞추기 위한 카운터 */
uint16_t uLSCnt = 0u;
/** @brief 속도 제어기 실행 주기를 맞추기 위한 분주 카운터 */
uint16_t uSpdCnt = 0u;

/* 파라미터 측정용 변수 실제 정의 */
volatile uint16_t uParamID_Start = 0;
volatile float fVd_Target1 = 0.4f, fVd_Target2 = 0.75f;
volatile float fVd_Test = 0.0f;
float fRs_Measured = 0.0f;
volatile uint16_t uParamID_L_Start = 0;
float fInj_Freq = 2000.0f;               // 주입 주파수 (2kHz)
float fVd_Inj_Amp = 0.2f;
float fTheta_Inj = 0.0f;
float fId_Max = -100.0f, fId_Min = 100.0f;
float fLd_Measured = 0.0f;


uint16_t uIdSubState = ID_READY;
static uint32_t uIdTimer = 0;
static float fIdSum = 0.0f;
static float fIdAvg1 = 0.0f, fIdAvg2 = 0.0f;

volatile uint16_t uIqPulseTestFlag = 0; // 이 값을 1로 만들면 펄스 시작
/**
 * @brief  20kHz 주기로 실행되는 메인 모터 제어 인터럽트 서비스 함수
 * @details
 * 1. CPU 사이클을 측정하여 제어 알고리즘의 연산 소요 시간을 모니터링합니다.
 * 2. 홀 센서를 통해 위치 정보를 업데이트하고, 과전압/과전류/과속도 Fault를 실시간으로 감시합니다.
 * 3. 상태 머신(State Machine)을 구동하여 IDLE, ALIGN, RUN, FAULT 상태에 맞는
 * 제어 로직(부트스트랩 충전, 위치 정렬, 전류 및 속도 제어, PWM 변조)을 호출합니다.
 * @param  없음
 * @retval 없음
 */
void vControl(void){	// 20kHz Interrupt (TIM1)


	ulControlStartClock = DWT->CYCCNT;

	//MOT1.SO.fThetarm = (fGetEncoderInfo(&htim3, &MOT1.SO));
	INV.SO.fThetar = -1.0f * (fGetHallSensorInfo(&INV.SO));


	/* fInvVdc 연산은 vScaleAdcValue()에서 수행되므로 중복 제거하여 루프 시간 단축 */

	////////////////////////////// State machine //////////////////////////////
	/* 하드웨어 및 소프트웨어 Fault 검사 (과전압, 과전류, 과속도 감지) */
	if((SW_Fault == 0u) &&
			((ABS(fVdc) >= VDC_FAULT_LEV) || (ABS(INV.CC.fIasHall) >= CURR_FAULT_LEV)
					|| (ABS(INV.CC.fIbsHall) >= CURR_FAULT_LEV) || (ABS(INV.CC.fIcsHall) >= CURR_FAULT_LEV)
					|| (ABS(INV.SO.fWrpmSC) >= SPD_FAULT_LEV))) {

		vSWFaultOperation();
	}
	else {}

	/* 리셋 명령 처리: 시스템 플래그 초기화 및 오류 해제 */
	if(Flag.RESET == 1u){
		Flag.START = 0u;
		Flag.RESET = 0u;

		uNextState = IDLE_STATE;
		vClearFault();
		vInitController();

	}else{}

	/* 상태 갱신 */
	uPrevState = uCurrState;
	uCurrState = uNextState;

	switch (uCurrState){
	case IDLE_STATE:
		if(uPrevState != IDLE_STATE){
			Flag.START = 0u;
			Flag.RESET = 0u;
			vInitController();
			vSwitchOffSettingTIM(&htim1);
			uBootStrapEnd = 0u;
		}
		else{}


		if (fVdc < 4.0f)					uNextState = IDLE_STATE;

		else if (SW_Fault || TZ_Fault) 		uNextState = FAULT_STATE;

		// 2. 정상 구동 시작 조건
		else if (Flag.START == 1u) {
			vBootstrapCharge(&htim1); // 부트스트랩 충전 수행

			if (uBootStrapEnd == 1u) {
				// 부트스트랩 완료 후, 제어 모드에 따른 상태 분기
				if (uControlMode == DUTY_TEST_MODE || uControlMode == CONST_VOLT_MODE) uNextState = RUN_STATE; // 위치 정렬이 필요 없는 모드: 바로 RUN 상태로 진입
				 else 	uNextState = ALIGN_STATE;	// 일반 FOC 등 위치 정렬이 필요한 모드
			} else 	uNextState = IDLE_STATE;	// 부트스트랩 충전 중에는 IDLE (또는 별도의 CHARGE_STATE가 있다면 그것을 사용)

		} else { // 3. 구동 정지 명령 시
			uNextState = IDLE_STATE;
			vSwitchOffSettingTIM(&htim1);
		}


		break;

	case ALIGN_STATE:
		if(uPrevState != ALIGN_STATE){
			vSwitchOnSettingTIM(&htim1);
		}

		vAlignHallSensor(&INV.CC, &INV.SO);
		vCurrentControl(&INV.CC, &INV.SO);
		vVoltageModulationTIM(&htim1, &INV.CC, &INV.SO);


		if (!Flag.START)  uNextState = IDLE_STATE;
		else if (INV.SO.uAlignEnd == 1) {
		    // 얼라인이 끝났을 때, 제어 모드에 따라 분기
		    if (uControlMode == ALIGN_MODE) {
		        uNextState = IDLE_STATE;
		        Flag.START = 0;
		    } else      uNextState = RUN_STATE;

		} else   uNextState = ALIGN_STATE;

		break;


	case RUN_STATE:
			if(uPrevState != RUN_STATE){
				vSwitchOnSettingTIM(&htim1);
			}

			if(uControlMode == PARAM_ID_MODE) {
			    switch(uIdSubState) {
			        case ID_READY:
			            fVd_Test = 0.0f;
			            if(uParamID_Start) {
			                if(fVd_Target1 > 0.5f) fVd_Target1 = 0.5f; // 안전 전압 제한
			                if(fVd_Target2 > 0.85f) fVd_Target2 = 0.85f;
			                uParamID_Start = 0;
			                uIdTimer = 0;
			                uIdSubState = ID_V1_SETTLE;
			            }
			            else if(uParamID_L_Start) {
			            	// 상저항 측정이 안 된 경우(0)를 위한 최소값 방어 로직//
			            	if(fRs_Measured <= 0.0f) fRs_Measured = 69e-3f;

                            fTheta_Inj = 0.0f;
                            fId_Max = -100.0f; fId_Min = 100.0f;
                            uIdTimer = 0;
                            uIdSubState = ID_L_INJECT;
                        }
			            break;

			        case ID_L_INJECT:

                        fTheta_Inj += 2.0f * PI * fInj_Freq * fTsamp;
                        if(fTheta_Inj > PI) fTheta_Inj -= 2.0f * PI;

                        fVd_Test = fVd_Inj_Amp * sinf(fTheta_Inj);

                        // 2. 피크 검출 (2000 count = 100ms 과도상태 무시)
                        if(uIdTimer > 2000) {
                            if(INV.CC.fIdsr > fId_Max) fId_Max = INV.CC.fIdsr;
                            if(INV.CC.fIdsr < fId_Min) fId_Min = INV.CC.fIdsr;
                        }

                        if(++uIdTimer > 12000) { uIdSubState = ID_L_CALC; } // 총 0.6초 측정
                        break;

                    case ID_L_CALC:
                        fVd_Test = 0.0f;
                        float fId_Amp = (fId_Max - fId_Min) / 2.0f;

                        if(fId_Amp > 0.05f) { // 노이즈 마진 확보
                            float Z = fVd_Inj_Amp / fId_Amp;
                            float omega = 2.0f * PI * fInj_Freq;
                            // L = sqrt(Z^2 - R^2) / omega
                            float L_sq = (Z*Z) - (fRs_Measured * fRs_Measured);
                            fLd_Measured = sqrtf(fmaxf(0.0f, L_sq)) / omega;
                        }

                        uParamID_L_Start = 0;
                        uIdSubState = ID_READY;
                        break;

			        case ID_V1_SETTLE:
			        	vSlopeGenerator(&fVd_Test, fVd_Target1, 10.0f * fTsamp); // 서서히 전압 증가
			            if(++uIdTimer > 20000) { uIdTimer = 0; fIdSum = 0.0f; uIdSubState = ID_V1_AVG; }
			            break;

			        case ID_V1_AVG:
			            fIdSum += INV.CC.fIdsr;
			            if(++uIdTimer > 20000) { fIdAvg1 = fIdSum / 20000.0f; uIdTimer = 0; uIdSubState = ID_V2_SETTLE; }
			            break;

			        case ID_V2_SETTLE:
			        	vSlopeGenerator(&fVd_Test, fVd_Target2, 10.0f * fTsamp); // 서서히 전압 증가
			            if(++uIdTimer > 20000) { uIdTimer = 0; fIdSum = 0.0f; uIdSubState = ID_V2_AVG; }
			            break;

			        case ID_V2_AVG:
			            fIdSum += INV.CC.fIdsr;
			            if(++uIdTimer > 20000) { fIdAvg2 = fIdSum / 20000.0f; uIdTimer = 0; uIdSubState = ID_CALC; }
			            break;

			        case ID_CALC:
			            if(ABS(fIdAvg2 - fIdAvg1) > 0.01f) {
			                fRs_Measured = (fVd_Target2 - fVd_Target1) / (fIdAvg2 - fIdAvg1);
			            } else {
			                fRs_Measured = -1.0f; // 전류가 안 흐르면 에러 표시(-1) 후 안전 종료
			            }
			            fVd_Test = 0.0f;
			            uIdSubState = ID_READY;
			           /* Flag.START = 0u; // 측정 종료 후 자동 정지 (좀비 상태 방지) */
			            break;
			    }
			}

			vSpeedObserver(&INV, &INV.SO, &INV.SC);

			// [중요] 제어 모드에 따른 각도 및 지령 연결 (정리된 버전)
			if(uControlMode == 1 || uControlMode == 2 || uControlMode == PARAM_ID_MODE) { // VECTCONTL_MODE
				// 홀 센서 각도에 '정렬 오프셋'을 더해줍니다.
				// (BOUND_PI는 각도를 -PI ~ PI 사이로 유지해주는 함수입니다)
				float fAngleWithOffset = BOUND_PI(INV.SO.fThetar + INV.SO.fThetarmOffset);

				vSinCos_Calculation(fAngleWithOffset, &INV.SO.fCosThetarCC, &INV.SO.fSinThetarCC);
				INV.SO.fCosThetarCompCC = INV.SO.fCosThetarCC;
				INV.SO.fSinThetarCompCC = INV.SO.fSinThetarCC;

				INV.CC.fIdsrRef = INV.CC.fIdsrRefSet; // 보통 0A
				INV.CC.fIqsrRef = INV.CC.fIqsrRefSet; // Live 창 입력값(2A)
			}


		if(uControlMode == CONST_CUR_MODE) {
		    static float fDummyTheta = 0.0f;
		    fDummyTheta += 2.0f * 3.141592f * 10.0f * fTsamp;
		    if(fDummyTheta > 3.141592f) fDummyTheta -= 2.0f * 3.141592f;

		    INV.SO.fThetar = fDummyTheta;

		    // 1. 제어용 각도 업데이트
		    INV.SO.fCosThetarCC = cosf(fDummyTheta);
		    INV.SO.fSinThetarCC = sinf(fDummyTheta);

		    // 2. [추가] 출력 재구성 및 Anti-Windup용 각도까지 똑같이 맞춰줍니다.
		    INV.SO.fCosThetarCompCC = INV.SO.fCosThetarCC;
		    INV.SO.fSinThetarCompCC = INV.SO.fSinThetarCC;

		    INV.CC.fIdsrRef = INV.CC.fIdsrRefSet; // 슬로프 없이 즉시 반영
		}

		if(uSpdCnt >= 39u){
			vSpeedControl(&INV, &INV.SO, &INV.SC);
			uSpdCnt = 0u;
		}

		vCurrentRef(&INV.CC, &INV.SC);

		if (uControlMode == 1) { // 1: VECTCONTL_MODE (벡터 전류 제어 모드)
		            // 위치 정렬 때 남은 2A를 지우고, Live 창의 0.0A를 연결
		            INV.CC.fIdsrRef = INV.CC.fIdsrRefSet;

		            // Live 창에서 입력한 1.5A를 진짜 Q축 지령으로 연결
		            INV.CC.fIqsrRef = INV.CC.fIqsrRefSet;

		            if (uIqPulseTestFlag && (uControlMode == 1)) {
             static uint32_t uPulseTimer = 0;
             if (++uPulseTimer > 10000) { // 0.5초 주기로 (20kHz / 10000 = 2Hz)
                 uPulseTimer = 0;
                 // 1.0A와 3.0A를 번갈아가며 지령으로 설정
                 if (INV.CC.fIqsrRef < 6.0f) INV.CC.fIqsrRefSet = 8.0f;
                 else                        INV.CC.fIqsrRefSet = 5.5f;
             }
         }

		        }

		if(uControlMode == CONST_CUR_MODE) {
				    // 1. D축 지령은 FOC의 정석대로 0A로 완전히 묶어버립니다.
				    INV.CC.fIdsrRefSet = 0.0f;
				    INV.CC.fIdsrRef = INV.CC.fIdsrRefSet;

				    // 2. Q축 지령은 Live 창에서 입력한 값(fIqsrRefSet)이 즉시 들어가도록 추가합니다.
				    INV.CC.fIqsrRef = INV.CC.fIqsrRefSet;
				}


		vCurrentControl(&INV.CC, &INV.SO);
		vVoltageModulationTIM(&htim1, &INV.CC, &INV.SO);

		if(SW_Fault || TZ_Fault)					uNextState = FAULT_STATE;
		else if(!Flag.START || (fVdc < 10.0f))		uNextState = IDLE_STATE;
		else										uNextState = RUN_STATE;

		uSpdCnt++;
		break;

	default: //case FAULT_STATE:
		vSwitchOffSettingTIM(&htim1);
		Flag.START = 0u;
		uNextState = FAULT_STATE;
		break;
	}



	vIntDacOut();
	uMainControl++;

	fElapsedTimeUs = (DWT->CYCCNT - ulControlStartClock) / fSysClkFreq * 1.0e6;
}



/**
 * @brief  2kHz 주기로 실행되는 저속 제어 루틴 (Low Speed Control)
 * @details 현재 내부에 실행 코드는 없으나, 온도 모니터링, 통신 처리 등
 * 20kHz보다 느린 주기로 실행되어야 하는 상위 제어 로직을 추가하기 위한 함수입니다.
 * @param  없음
 * @retval 없음
 */
/// 2kHz Interrupt ///
void vLowSpdControl(){

}
