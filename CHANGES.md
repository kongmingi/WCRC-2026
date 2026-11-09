# 변경 기록

## 남은 일 (테스트 메모)
- t5 3: 1열 중앙을 잘 못 잡음. t5 3은 1열까지 가는 것만 함(스캔·집기는 t4, t6)
- t5 4: 동작은 맞는데 Zone A 위치가 애매함
- t4 테스트 H: 3행(L2) 빨강·파랑 놓침 → scan2에서 t1 x,y 확인 필요
- 6핀 케이블 납땜 부위 5V/GND 불량 의심 (노트북 USB 연결 시에만 Pixy 켜짐)

## T30 (팀원 WCRC_T26_updated 기준, 실기 OK: t7 6개 154초)
- 이동 속도 140, 90° 회전 ROTATE_MM_PER_SEC 100, B 갈 때 왼쪽 회전 +2°, POSE_ORDER_R { 80, 2007, 3929, 932 }
- T27~T29 내용 없음, 순서 불확실하면 경기 중단(팀원 코드)

## T31 (T30 + T27~T29 다시 넣음, 아직 실기 확인 전)
- 팀원 변경: 순서 읽기 4번 촬영(왼쪽 켬/끔, 오른쪽 끔/켬) + x 로 합치기, SW2 정지 후 상태 초기화·팔 복귀
- 순서가 불확실하거나 개수가 안 맞으면 경고만 하고 진행 (팀원 코드의 경기 중단 제거). 못 읽은 블록은 배정표 순서로 뒤에
- 순서 읽기: 같은 색 조각(간격 20px 이하) 합치고 폭 115 이상이면 2개 (T27)
- B 복귀 왼쪽 150 + SL 572 (T27), 아는 칸 건너뛰기·집은 칸 x (T27)
- PSD 동시 맞춤 최소 속도 (T28), 이동 시간 초과 시 정지 + `[MOVE] timeout` (T29, Mobilebase.cpp)
- 이동 속도 140, 90° 회전 ROTATE_MM_PER_SEC 100, B 갈 때 왼쪽 회전 +2°, POSE_ORDER_R { 80, 2007, 3929, 932 }
- Tuning 추가: ROTATE_MM_PER_SEC, B_GO_TURN1/2_ADJ_DEG, ORDER_JOIN_WIDTH_PX, ORDER_MERGE_GAP_PX, mm_L_B_RETURN_TO_REF

## T29 (T28 개선, T28 유지)
- 이동 시간 초과면 바퀴 즉시 정지(목표 취소) + `[MOVE] timeout` 출력, 전송 실패도 출력. Mobilebase.cpp
- Tuning: 이동 속도 140 (MOBILEBASE_SPEED, MOVE/SIDE/SEARCH_MM_PER_SEC)

## T28 (T27 개선, T27 유지)
- PSD 앞뒤+비틀림 동시 맞춤: 허용 밖인데 속도 10 미만이면 10으로 (멈춘 채 헛도는 문제). Mission2B.ino `RotateAlignFront2()`. 값 변경 없음

## T27 (T26 개선, T26 유지)
- 순서 읽기: 같은 색 덩어리가 붙어 있으면(간격 20px 이하) 한 묶음, 전체 폭 115px 이상이면 2개 / 미만이면 1개 (반사로 쪼개진 조각 합침)
- 읽은 개수가 zone 개수와 다르면 `[ORDER] WARNING` 출력
- Zone B 복귀: 2열에서 왼쪽 150만 가고 나머지는 SL 572 맞춤으로 (Zone A 충돌 방지)
- 일반 탐색: 다른 색이 기록된 칸, 집어서 비운 칸(x)은 건너뜀. "안 보였음(-)" 칸은 다시 봄
- Tuning 추가: ORDER_JOIN_WIDTH_PX 115, ORDER_MERGE_GAP_PX 20, mm_L_B_RETURN_TO_REF 150

## T26 (T25 개선, T25 유지, 값 변경 없음)
- 적재함 기억: 스캔 때 ROI 안 가장 큰 블록 색을 칸[열][층]에 기록(5프레임 중 3 이상), 블록 없으면 빈칸. 집으면 빈칸
- 다음 블록이 기록에 있으면 1열에서 그 열로 바로 가서 그 층부터 확인 → 없으면 기록 지우고 1열로 와서 보통 탐색
- 시간 출력 `[T] 구간 +구간초 total 누적초`: obstacle, order read, col1 arrive, pick col N, drop A/B N, back to col1, not found, FINISH
- 경기 끝나면 `[INV]` 지도 출력. 콘솔 `inv`, `inv clear`

## T25 (T24 개선, T24 유지)
- Zone B 복귀: B → 2열(FR 304 + 비틀림) → 왼쪽 185(70 + 115)로 A 앞 → SL 572 + 벽 맞춤 → 오른쪽 115 → 1열 (A 복귀와 같은 마무리)
- Tuning: za1~8, zb1~8 다시 잡은 값

## T24 (T23 개선, T23 유지)
- 순서: 카메라가 읽은 순서대로 진행. 같은 색은 배정표 위쪽 줄 칸부터. 배정표에 없는 색은 건너뜀, 안 쓰인 배정표 줄은 맨 뒤에 붙임
- LED: 찾기 시작할 때 찾는 색으로 켬 → Zone A/B 에 놓으면 끔
- zone 명령: `zone` 보기, `zone 빨 A1 빨 B5 노 A4 ...` EEPROM 저장(업로드 없이 반영), `zone clear` → Tuning.h 표로
- Tuning: RED~PURPLE, ZA(n), ZB(n) 로 배정표 쓰기 (숫자 형식도 그대로 됨)

## T23 (T22 개선, T22 유지)
- t7: t5 1 → t5 2(팀원 방식: 이동 팔 자세 → 도착 후 orderl → 읽기, 램프 Finish까지 켬) → t5 3(ArriveCell1) → 적재함 탐색
- SW1 = t7, SW2 = 정지 (바퀴 토크 끔 → 대기 상태로 복귀, 콘솔 명령 중에도 동작)
- Mobilebase.cpp: 이동 대기 중 정지 버튼 확인

## T22 (팀원 코드 기준, T20 유지)
- 원인: SL 607/611 이 PSD 최고점 평평한 구간(608~611, 약 15mm)이라 Zone A 앞 위치가 매번 달라짐 → 1열 dx 틀어짐
- t5 3 / Zone A 복귀: SL 572(기울기 구간) → 벽 맞춤 → 오른쪽 115 → 1열
- Zone A 갈 때: 좌진 기준 130 → 115, SL 572 → 왼쪽 7 → 놓기
- t5 2 orderLeft 자세 대입 오타 수정 (orderRight 두 번 → orderLeft)
- Tuning: PSD_DROPSIDE_REF_SL 572, mm_R_REF_to_CELL1 115, mm_L_REF_to_DROP 7
- 그대로: t7 1열 도착(강사님 순서), Finish SL 611
- 열 간격(실측): 오른쪽 73 / 75 / 67, 왼쪽 67 / 75 / 70 숫자 고정, B 복귀 전진 139 숫자 고정

## T20 (T19 개선, T19 유지)
- 순서 읽기 ROI: 블록 중심이 박스 안일 때만 (orderL x 22~206 y 43~168, orderR x 84~316 y 92~185)
- Tuning: ORDER_ROI[2][4] = { x최소, x최대, y최소, y최대 }

## T19 (T18 개선, T18 유지)
- t5 2 / t7: 출발할 때 orderl 자세로 가서, 도착하자마자 왼쪽 읽기 (팔 이동 시간 없음)
- 순서 읽기: 칸 기준 x·y 범위 삭제, 화면 전체 블록 x 정렬 → 왼쪽 자세는 왼쪽부터 3개, 오른쪽 자세는 오른쪽부터 3개
- Tuning: ORDER_SLOT_X, ORDER_Y_MIN/MAX 삭제

## T18 (T17 개선, T17 유지)
- orderr: 칸 기준 x·y 범위 없이 보이는 블록 왼쪽부터 3개, 5프레임 중 같은 순서 3번 이상
- Tuning: ORDER_L {-256,1872,4236,1330}, ORDER_R {129,1780,4333,1328}, 왼쪽 칸 x {90,148,240}, y 0~207

## T17 (T16 개선, T16 유지)
- 미션 순서: 왼쪽(orderl, 35) / 오른쪽(orderr, 36) 두 자세로 촬영, 칸 중심 x 6개에 블록이 걸치면 그 색 투표 (5프레임 중 3 이상)
- 같은 색 붙은 블록, 화면 끝 옆 칸 블록 처리. 빈칸은 0 으로 출력 후 건너뜀
- 출력 `[ORDER] slots: 1 2 3 4 5 6`
- Tuning: POSE_ORDER_L/R, ORDER_SLOT_X, ORDER_Y_MIN/MAX, ORDER_FRAMES, ORDER_MIN_VOTES, ORDER_POSE_MS, ORDER_POSE_SETTLE_MS

## T16 (T15 개선, T15 유지)
- Zone B 복귀: 뒤로 366 → 오른쪽 90° → 앞으로 139 → 오른쪽 90° → 2열 맞춤(FR 304 + 비틀림 -5) → 왼쪽 70 → 1열 FR 304 (강사님 FR 194·SR 275·SL 200 맞춤 뺌)
- Tuning: B_RETURN_TURN1/2_ADJ_DEG 0, mm_F_DROPBOX_B_to_CELL2 = 67 + 75 - 3
- Zone B 값: 1열 왼쪽 회전 뒤 300 / -2, Zone B 정면 554 / -29, zb1~8

## T15 (T14 개선, T14 유지)
- A1 에 놓을 때만 뒤로 mm_B_ZONE_A1(10) → 놓기 → 앞으로 10
- Zone A 놓은 뒤 복귀: 이동 자세 → SL 607 → 벽 맞춤 → 오른쪽 123 → 1열 FR 304 (t5 3 끝부분과 같음, ReturnToCell1FromDropside)
- (T14 에서 이어짐) POSE_CARRY, za1~8, 놓는 거리 130, 놓는 SL 611

## T14 (T13 개선, T13 유지)
- 운반 자세 분리: POSE_CARRY { -11, 1488, 4447, 1498 } = 4번 (집은 뒤·Zone 이동, 밑동은 경기 중 정면/왼쪽 강제). 출발·순서 읽기는 POSE_TRAVEL 그대로
- 콘솔: p carry / save carry 추가, save travel 은 travel 만 바꿈, list 에 travel·carry 둘 다
- (T13 에서 이어짐) col N 명령, pg2 { 1013, 1855, 4510, 2260 }

## T13 (T12 개선, T12 유지)
- 2·3열 비틀림 목표를 열마다 따로: PSD_STORAGE_COL_ROT_ADJUST[4] = { 0, -5, -11, 0 }, 허용 3 (공용 -8 / 4 일 때 3열이 기울어 dx +17)
- t5 3 N, t4, 경기 탐색, B 복귀, 4열 집은 뒤 2열 보정 모두 그 열 값 사용
- (T12 에서 이어짐) 열 간격 70 / 75 / 70, pg3 { 1014, 1910, 3971, 1777 }

## 정리 (main 은 T25 부터)
- T9~T24 도 `archive` 가지로 옮김
- v6~v28, T1~T8, 옛날 PDF·계획 문서는 `archive` 가지로 옮김 (GitHub 가지 선택에서 archive)

## T12 (T11 개선, T11 유지)
- t5 1 / 경기: 장애물 앞 FL·FR 로 앞뒤+비틀림 (PSD_OBSTACLE_AVG 562, PSD_OBSTACLE_ROT_ADJUST -25, 허용 3)
- 오른쪽 이동 129 → 123
- 램프: CAM_LAMP_ON true, t5 2 시작할 때 켬 (순서 읽은 뒤 꺼짐), t5 3 1열 FR 맞춤 끝나면 켬

## T11 (T10 개선, T10 유지)
- 바퀴 비율: 왼쪽 { 0.99, 0.99, 1, 1 } (왼쪽 이동 때 왼쪽 바퀴가 뒤처짐), 오른쪽 같은 값 (수평 확인)
- 오른쪽 이동 122 → 129 (비율 변경 후 1열까지 7mm 모자람)

## T10 (T9 개선, T9 유지) — 블록 집기 시작 기준
- T9 마지막 상태 그대로: t5 3 (전진 → SL → 벽 → 오른쪽 122 → 1열 FR 304) 확인 완료
- 카메라: 그리퍼 중심 x 층별 { 185, 182, 177 }, 0.48 mm/px (move R 20 = 42 px)

## T9 (T8 개선, T8 유지)
- (수정) t5 3 순서: 전진 → SL → 벽 맞춤 → 오른쪽 122 → 1열 FR 304 (벽 맞춤을 SL 뒤로: 좌우 위치에 따라 FR-FL 이 달라 비틀리던 문제)
- (수정) 그리퍼 중심 x 층별 PIXY_SET_POINT_X[3] = { 185, 178, 177 } (1·2·3층), 카메라 보정·t3·t1 dx 가 보는 층 값 사용
- (수정) t5 3 = 이동 자세 → 전진 → 벽 맞춤(FL·FR) → SL 맞춤 → PSD 출력 후 멈춤 (오른쪽 이동·1열 정렬 뺌, 경기 t7 은 그대로)
- (수정) MAX_MOV_ALIGN_REPEAT_COUNT 3 → 10 (맞춤 최대 약 0.4초 → 1.3초)
- t5 2 / 경기: 순서 읽기 자세 = 출발 자세(34, POSE_TRAVEL 그대로), 뒤로 20
- t5 3 / 경기: 전진 → 앞쪽 벽 맞춤(PSD_SEARCH_WALL_AVG 190, FR-FL 0, 허용 3) → 탐색선 SL 607 ×2 → 오른쪽 → 1열 FR
- pa3 { 1014, 1534, 4117, 1549 }

## T8 (T7 버그 수정, T7 폴더 삭제)
- p travel = 출발 자세(34, 밑동 그대로), save travel 은 4·34 같이. 순서 읽기(27) = POSE_ORDER_VIEW 로 분리
- t5 2 / 경기: 왼쪽 → 뒤로 mm_B_OBSTACLE_to_MISSION → SL
- t5 3 / 경기: 이동 자세(3, POSE_DRIVE)로 전진, 1열 FR 까지 그 자세 유지
- check 에 바퀴 모터 1~4 전압 표시
- Tuning.h 사용자 값 반영: 속도 100, 바퀴 비율 1.000, 왼쪽 350 / 뒤로 50, 오른쪽 85, FR 304, travel / order / drive 자세

## T7 (T6 버그 수정, T6 폴더 삭제)
- T6 팔 값이 전부 틀어짐 (travel 이 -1010, 1206, 2244, 890 으로 읽힘): 강사님 위치 모드 + 오프셋으로 변환하는 방식이 이 로봇에서 안 맞음
- 팔 구동을 우리 코드(v28)와 같게: 확장 위치 모드, 오프셋 0, 재부팅, 관절 창 계산, P/I 게인 1200/200 → Tuning.h 자세 숫자가 그대로 모터 값
- 집기·운반 때 밑동 고정은 `J1_FRONT` 1013 기준 (강사님 0° = 정면, -90° = 1013 - 1024)
- Tuning.h: ARM_OFFSET/ARM_POS_MIN/MAX 삭제, ARM_P_GAIN, ARM_I_GAIN, J1_FRONT 추가

## T6 (T5 개선, T5 유지)
- `Tuning.h` 추가: 스위치, 미션 배정, 속도, 바퀴 비율(WHEEL_RATIO_*), PSD, 거리, 카메라, 그리퍼(GRIP_OPEN/CLOSE), 팔 범위·오프셋, 팔 자세 전부
- 팔 자세: v28 Tuning.h 값 그대로(우리 좌표), EEPROM 대신 Tuning.h 에서 읽어 강사님 좌표로 변환 (ARM_OFFSET, JOINT_MIN/MAX 창)
- 이동·PSD 위치 잡기 값은 강사님 값 그대로
- 콘솔: pose/j1/arm 이 Tuning.h 숫자로 표시·입력, save = 전원 끄기 전까지 + Tuning.h 에 옮길 줄 출력, list, p hub. ours/del 삭제
- Devices/Mobilebase/Manipulator 는 값만 Tuning.h 에서 읽게 변경, 동작 동일

## T5 (T4 개선, T4 유지)
- 스케치 1개(`T5/Mission2B`)에 강사님 경기 + 우리 콘솔 명령 (help pose free lock jN arm grip p move fr sl psd check t1~t7), 새 명령 save del list ours sr rot turn w wf baud, t5 3 N(열 이동), t4 색(찾아 집기)
- SW1 또는 t7 = 강사님 전체 미션. 미션 배정은 missionAssignments 직접 입력 (order/zone 명령 없음)
- 오류·종료 시 멈추지 않고 콘솔로 복귀 (gameOver)
- 밑동 오프셋 1034 (정면 1013), 그리퍼 740 / 250, ours = v28 자세 (za 는 예전 밑동 기준 380 으로 변환, zb 없음)

## v28 (v27 개선, v27 유지)
- 밑동 정면 1013 기준으로 다시 잡은 값: travel { -14, 1660, 4106, 1230 }, order·scan·pa·pg 밑동 1013 고정
- 코드 변경 없음 (Tuning.h 만)

## v26 복구 (커밋 0e29a7b 그대로, 강사님 시연용)

## T4 (T3 개선, T3 유지)
- 1번 관절 오프셋 `ARM_DXL_1_OFFSET` 0 → 380 (우리 로봇 정면 1667 → 2047), Mission2B·Tool 둘 다
- Tool `ours`: v27 팔 자세 23개를 강사님 오프셋 { 380, 238, 799, 0 } 으로 변환해 EEPROM 저장 (B 19~26 없음)
- 4·34 travel, 27 order, 33/32/31 scan1~3, 8·9·10 / 5·6·7 / 28·29·30 = pa·pg·pa (1~3), 11~18 za1~8

## T3 (T2 개선, T2 유지)
- 도구 6개를 `T3/Tool/Tool.ino` 하나로 합침, 짧은 명령 (r t p s x l o c g psd cam f b lf rt rl rr fa sa ra ma k d w wf wb wl wr id baud)
- 자세를 명령 `s N` 으로 EEPROM 에 바로 저장 (writeEEPROM 표 복붙 불필요, 형식 같음)
- 바퀴 비율을 실행 중 배열(`wheelRate`)로 사용 → Tool 에서 `wf` 등으로 바로 바꿔 시험. 기본값은 그대로 Mobilebase.h
- 블루투스(Serial2)로도 Tool 명령 가능
- Mission2B 동작·값 변경 없음

## v27 (v26 수정, v26 폴더 삭제)
- t3 집기 순서 단순화: scan(확인) → 그리퍼 열기 → pa → pg → 닫기 → pa → travel
- hub 전부 삭제(집기), 뒤로 10mm 삭제(`PICK_FWD_MM` 삭제)
- 카메라 좌우 중앙 맞춤 끔 (`CENTER_ENABLE` = false, 켜면 다시 동작)

## v26 (v25 수정, v25 폴더 삭제)
- 적재함부터(t5 3 이후) 앞뒤 PSD 보정 전부 삭제: 1열 도착, 스캔 열 이동, 집기 전·중·후, Zone B 출발, Zone 복귀
- 남은 PSD: t5 1 FR(앞뒤, 579), 1열 SL(좌우, 258), Zone A SL(좌우, 634). t5 2 SL·Zone B SR 은 0(꺼짐)
- 적재함 선반 감지 전진(ApproachFront) 삭제

## v25 (v24 버그 수정, v24 폴더 삭제)
- 잡기 순서에서 pa 앞의 hub 삭제: pa → pg → 뒤로 10 → 잡기 → pa → travel

## v24
- t5 3: 오른쪽 40, 1열 기준 FR 308 + SL 258 (정중앙에서 잰 값)
- 집은 뒤: pa → travel (hub·order 안 거침, 블록 걸림 방지)
- 스캔: 열 이동이 끝난 뒤 앞 거리 맞춤 (열마다 앞으로 밀리던 문제). 1~3열 FR, 4열 FL
- ROI 임시값: y 60~190 / 60~185 / 60~172

## v23
- 1열 기준: FR 312(앞뒤) + SL 269(좌우, Zone A 상자) 켬, 방향 보정 끔
- pa/pg 1~3 새 값 (1=2행 맨 아래, 3=4행 맨 위)

## v22
- 다른 컴퓨터 값 반영: order 자세 { 1626, 1138, 4483, 1314 }, t5 2 SL 맞추기 끔
- 잡기: hub → pa(칸 앞, 같은 높이) → pg(천천히) → 뒤로 10 → 잡기 → pa → hub (`POSE_PICK_APPROACH`, `pa1~3` 명령)
- 중앙 맞춘 뒤 팔을 hub 로 접고 앞 거리 맞춤 (scan1 자세에서 팔이 FR 센서를 가려 631 이 읽히던 문제)
- 경기장 1열: 900mm 전진 + 오른쪽 35, FR 300, SL 맞추기 끔, 방향 보정 끔
- scan1~3, pa/pg 1~3, 기준 x { 153, 159, 161 } (경기장 적재함)
- t5 2 뒤로 60

## v21
- 실 경기장 적재함: 1열 앞 거리 FR 189, 1열 SL 172, 열 간격 72mm
- 잡기: pg 자세 → 뒤로 10mm (`PICK_FWD_MM` -10, 음수=뒤) → 잡기 → hub → 앞 거리 복귀 (PSD 잡는 깊이 `PSD_GRIP_FR_ADC` 삭제)
- 로봇 Tuning.h 반영: pg 자세, order 자세 { 1611, 1343, 4639, 1810 }
- travel { 671, 1878, 3714, 930 }, 미션지시 Zone 뒤로 40

## v20
- 실 경기장 값: 라인 440mm + FR 579, 미션지시 Zone 왼쪽 270 → 뒤로 60 → SL 397
- travel 자세 { 616, 2078, 3697, 1025 } (순서 6개 인식 자세)
- t5 2: 왼쪽 → 뒤로 (`DIST_LINE_TO_ORDER_BACK_MM` 추가)

## v19
- 명령 재구성: `t6` = 1열에서 시작해 입력 순서대로 찾기·집기·배달, `t7` = 전체 미션
- 시리얼 입력: `order 5 1 6 3`, `zone A 1 5`, `list`, `clear` (미션 표 MISSION 삭제)
- 이어서 하기: 미션 중 정지 후 SW1 또는 `go` → 멈춘 단계부터 (집은 블록·스캔한 칸·남은 순서 기억)
- 방향 보정: 2열에서 FL − FR = 0 이 되게 제자리 회전 (적재함 도착, Zone 복귀, 스캔 중 2열). `rot` 명령, `HEADING_*` 값
- t5 2: travel 자세로 순서 인식 → order 자세. 인식 실패 시 입력한 순서 사용
- t5 3: order 자세 유지한 채 전진 → 1열 정렬(+방향 보정)
- order 자세 v15 값 { 1659, 1143, 4469, 1245 }
- Zone B: 3·4열에서 집으면 1열 거치지 않고 바로 배달
- 부저: 순서 인식 성공 1번 / 실패 3번 / 집기 실패·블록 없음·배정 없음 2번 / 시간 초과 길게
- PSD: 21번 읽기, 허용 ±2, 최소 속도 5

## v18
- 바퀴 모드 바꿀 때 목표값 초기화 (PSD 맞춘 뒤 다음 이동에서 뒤로 튀던 버그)
- 순서 읽기: 미션 표와 상관없이 카메라가 읽은 순서 그대로 사용 (5장 중 2장 이상 같으면 확정)
- t5 1: 330mm 전진만 (`PSD_LINE_FR_ADC` 0)
- t5 2: 왼쪽 330mm만 (`PSD_ORDER_SL_ADC` 0), order 자세 { 633, 1950, 4119, 1544 }
- travel 그대로 { 631, 1469, 4278, 1344 }

## v17
- 블루투스 시리얼 추가: USB(Serial)와 블루투스(Serial2, 115200) 동시 사용. 출력은 양쪽, 명령은 어느 쪽에서든 입력
- 값 변경 없음

## v16
- 로봇 Tuning.h 그대로 반영 (Zone A 자세·경로, order 자세, 램프 끔)
- 라인: 330mm 전진 + FR 536 맞춤
- 미션지시 Zone: 왼쪽 340, SL 495, order 자세 { 454, 1459, 4453, 1351 }
- 미션지시 → 적재함: 전진 뒤 오른쪽 94mm 추가 (`ORDER_TO_COL1_RIGHT_MM`, 실측 필요)
- Zone A 앞뒤: `ZONE_A_FWD_MM` 음수면 뒤로

## v15
- Zone B: 1열 → 4열(FL 183) → 오른쪽 30 → 후진 350 → 놓기 → 1열 복귀, `zb1~8` 자세
- 관절 범위: j1 -1500~2600 (팔이 뒤로 짧게 돎), j3 최소 2650
- 옆 이동 속도 제한 `SPEED_SIDE_MMPS` 100
- 명령 `fl 183`, `t5 4 칸`, `t5 5 칸`

## v14
- 바퀴 모드 변경 3회 재시도 + 확인 (한 바퀴 안 도는 문제)
- Zone A 앞 30, 복귀 오른쪽 150

## v13
- Zone A 경로(왼쪽 → 앞 → SL 맞춤)와 `za1~8` 자세
- 명령 `sl 637`, `t3` 소문자·잘못된 번호 처리
- 스캔: 4행 y 상한 140, 대기 600ms, 스캔 중 앞 거리 맞추기 삭제

## v12
- 층별 ROI(y 최소·최대), x 범위 ±45, 기준 x { 170, 171, 163 }
- 3·4열은 FL로 앞 거리 맞춤

## v11
- 잡기: FR 183 → pg 자세 → FR 219 전진 → 잡기 → 183 후진
- 명령 `fr 183`

## v10
- Pixy 통신 `Pixy2`(Arduino ICSP SPI)

## v8 ~ v9
- pa/po 자세 삭제, 바퀴 전진으로 잡기

## v7
- 팔 동작 부드럽게(가감속), hub 자세, 층별 스캔, P/I 게인

## v6
- 팔 확장 위치 모드 + 관절 창 계산 (좌표 튐·바닥 박힘 해결)
