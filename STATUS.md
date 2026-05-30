# hand_ws/src — 작업 STATUS 로그

> 세션 disconn 대비 작업 기록. 새 작업/디버깅마다 위쪽에 append. 메모리 회생 시 이 파일 먼저 읽기.

---

## ▶▶ RESUME HERE (2026-05-30 기준) — 새 세션이면 이것만 먼저 읽고 시작

### 한 줄 요약
**왼손은 벤더 수리 보냄(못 씀)**, **오른손은 모터 동력 전원이 안 들어온 것으로 강하게 의심**. 사용자 물리 점검 답변 대기 중.

### 양 손 현재 상태
**왼손 (IP 169.254.186.73):**
- Tesollo 벤더 수리 보냄. 두 관절 결함 — `lj_dg_1_4` (엄지 DIP, 전기 결함 + 발열 — 코일 단락 또는 드라이버 MOSFET 의심), `lj_dg_3_2` (중지 MCP, 모터/드라이버 채널 사망 — 가끔 첫 시도만 작동).
- 수리 동안 사용 금지.
- 시리얼번호 — 사용자가 아직 안 줌. 받으면 메모리 `dg5f_hardware_ips` 에 추가.

**오른손 (IP 169.254.186.72):**
- ping OK, 드라이버 띄우면 LED 점멸→solid (Modbus 통신 정상), 컨트롤러 `rj_dg_pospid` active, 모든 효과인터페이스 `available claimed`.
- 그러나 **물리적으론 토크 없음**: 손가락이 "driver-off 같은 cogging 만" 느낌, `joint_states.position` 가 **항상 0.0** (인코더 무전원 의심), finger_diag 가 명령은 정상 publish 하는데 핸드 안 움직임.
- **확정 가설:** LED solid = **로직 보드(Modbus) 만** 살아있고, **모터 동력 전원이 OFF**. DG5F 는 로직↔모터 동력 분리 가능성.

### 사용자에게 받을 답 (대기 중) — 재접속 시 이걸 받아서 진행
오른손 본체/전원 물리 점검:
1. 케이블/플러그 갯수가 왼손이랑 같은가? (모터용 굵은 전원선 따로 있나)
2. 핸드 본체에 별도 전원 스위치/버튼/E-stop 같은 게 있나?
3. 모터 전원 표시 LED(작은 빨/녹, Modbus 파랑 LED 외)가 있나? 켜져 있나?
4. 두 손이 같은 PSU 의 다른 채널 쓰면 오른손 채널 OFF 여부?
5. (가능하면) 24V 라인 멀티미터 측정

이 답에 따라:
- 전원 OFF 발견 → 켜고 끝. 그담 finger_diag 정상 돌릴 수 있음
- 전원 다 OK 인데 증상 동일 → 오른손도 모터 보드/파워 모듈 고장 → 벤더 같이 보내야 할 수도

### 최근 push 된 코드 (둘 다 적용 상태)
- **ss_prep_260417 / main:** `7a8f04d` NaN 가드, `7fdc529` 글러브 side 기반 선택(인덱스 스왑 무관), `3bfc231` dg5f_finger_diag 패키지, `d644283` 1_4 override 롤백(왼손 수리 보내서)
- **LeonJung/dg_hardware (fork) / main:** `8faba48` 종료시 zero-duty 토크 릴리즈, `45d26c7` 중복 SendDuty 가드 (로그 노이즈 제거)
- **테스트 머신 빌드 필요 패키지:** `manus_dg5f_retarget manus_dg5f_grasp_mode manus_dg5f_sota_retarget_a manus_dg5f_sota_retargeting_a_good dg5f_finger_diag delto_hardware`

### 작업 환경 메모
- 사용자는 **Claude 가 도는 머신이 아닌 다른 머신**에서 테스트. 진단 커맨드는 사용자가 그쪽에서 직접 돌림.
- 핸드손가락 별칭: a=엄지 b=검지 c=중지 d=약지 e=새끼. 슬롯 0-3 / 4-7 / 8-11 / 12-15 / 16-19.
- 컨트롤러: 왼손 `lj_dg_pospid`, 오른손 `rj_dg_pospid` (둘 다 `pid_controller/PidController`, 20 joint MultiDOF).
- 토픽: `/dg5f_<side>/<lj|rj>_dg_pospid/reference` (control_msgs/MultiDOFCommand), `/dg5f_<side>/joint_states`.
- Manus 글러브는 연결 순서로 `/manus_glove_0`/`_1` 배정 (세션마다 좌/우 뒤바뀜). side 필드로 식별 — 우리 노드들 둘 다 구독 후 side 필터로 선택 (커밋 `7fdc529`).

### 이 세션에서 해결됐던 큰 줄기들 (참고)
1. 시작: grasp_mode 안 움직임 → mock+sim 기본값 함정 + 글러브 NaN + Manus 인덱스 스왑(side 식별로 해결)
2. 왼손 발열 사고 → NaN 가드 추가, 글러브 재캘리브
3. 종료시 손 굳음 → fork 에 on_deactivate zero-duty
4. finger_diag 패키지 신설 → 왼손 1_4·3_2 결함 확정 → 벤더 수리 결정
5. 오른손 검증 → 모터 전원 OFF 의심 (← 현 위치)

---

## ▶ 현재 진행 (2026-05-29)

**실물 점등 확인:** 전원 누락이었음(케이블/페이로드 아님). 전원 ON 후 핸드 동작 확인.

**새 패키지 `dg5f_finger_diag` 추가 (커밋 `3bfc231`, push 완료):** 관절 20개 자가진단. 모드 step(기본, baseline+0.3rad 스텝→측정→복귀) / passive(명령 없이 N초 관측). verdict: PASS / WARN_TRACKING / WARN_HIGH_EFFORT / FAIL_STUCK / FAIL_REVERSED / FAIL_POOR_TRACK. teleop 과 동시 실행 금지(reference 토픽 충돌).
- 테스트 머신: `git pull` + `colcon build --packages-select dg5f_finger_diag` + source.
- 실행: 드라이버 띄운 채로 `ros2 run dg5f_finger_diag finger_diag --ros-args -p hand_side:=left` (mode:=passive 도 가능).
- 의심 1순위: slot 17(lj_dg_5_2, 새끼 MCP) — 편 자세 0.41rad offset.

**최종 결정 (2026-05-29): 왼손 벤더 수리 보냄. 수리 기간 안 쓸 예정.**
- **롤백 push `d644283`** (ss_prep): 위 lj_dg_1_4 PID=0 override 제거. 수리 후 돌아와서 헷갈리지 않게 기본 게인 그대로 유지. 그동안 왼손 사용 금지.
- **fork `45d26c7`** (LeonJung/dg_hardware) 그대로 유지: 로그 노이즈 제거뿐이라 수리 전/후 무관.

**[참고용 기록] 발열 차단 override 패턴** (수리 후 또 동일 증상이면 재적용):
- 파일: `manus_glove/manus_dg5f_grasp_mode/config/torque_overrides_left.yaml`
- 추가: `lj_dg_1_4: { p: 0.0, i: 0.0, d: 0.0, i_clamp_max: 0.0, i_clamp_min: 0.0 }`

**Fork 토크 릴리즈 검증 ✓ (사용자 보고):** drv 런치 Ctrl-C 시 손 늘어짐(cogging만, "driver-off 느낌"), 다른 관절 정상. 1_4·3_2 만 별도 결함.

**남은 작업:**
- 벤더(Tesollo) 점검 의뢰 — 1_4 (모터/드라이버, 발열), 3_2 (모터/드라이버 채널 사망)
- finger_diag 가 쓰는 단독 driver 런치엔 override 안 들어가니 1_4 step_sign=0 으로 우회 (또는 그 런치도 fork 처리, 보류)

**진단 2차 (2026-05-29, 사용자 수동 점검):**
- **1_4 엄지 DIP: 전기적 결함 거의 확정 + 발열 = 긴급.** 손으로 만져본 거동 — 일직선에서 한쪽으로 조금씩 curl, 막으면 정지, 밀어주면 그쪽으로 빨려들어감. PID 가 positive feedback 된 전형. 원인: 인코더↔모터 방향부호 미스매치 또는 commutation/홀 결함. **계속 두면 코일 손상.** → SW 임시방편: 그 관절 PID gains=0 (torque_overrides_left.yaml 에 lj_dg_1_4 블록 추가).
- **3_2 중지 MCP: 간헐 모터/드라이버 결함.** 전원 사이클 후 첫 시도 가끔 됨, 대부분 토크 0. 발열 없음 — 비활성 불필요, 자유 상태로 두면 됨.
- 전원 OFF: 손가락들 완전 부드럽 = 기구 정상.
- 전원 ON + 노드 없음: 살짝 뻑뻑(cogging 정도) = 토크 비활성 상태 정상.
- **벤더 점검 필요** (Tesollo): 1_4, 3_2 둘 다.

**진단 1차 결과 (2026-05-29 step):**
- **FAIL_STUCK `lj_dg_1_4` (엄지 DIP):** base=-0.044, motion=-0.003, peakE=20.0 → 모터 전류 흐르는데 안 움직임 = 기계 잠김(텐던/걸림) 또는 모터 전기 고장. **물리 점검 필요** (Ctrl-C 로 토크 풀고 손으로 굽혀 보기).
- **FAIL_STUCK `lj_dg_3_2` (중지 MCP):** base=**1.388rad(≈80°)**, peakE=0 → 모터 명령 자체가 안 감 + base 비현실적. 인코더 오프셋/스턱 또는 모터 드라이버 채널 고장 의심. **passive 재측정 필요**: Ctrl-C → 중지 펼친 자세로 두고 다시 passive 돌려 slot 9 pos_mean 확인. 여전히 1.388 → 인코더 사망.
- **WARN_HIGH_EFFORT 9개** (1_1,1_3,2_1,3_4,4_1,4_3,4_4,5_1,5_3): 임계 기본 30 이 보수적. `peak_eff_high:=60` 으로 재실행 후 남는 것만 실제 문제로 추적.
- **FAIL_POOR_TRACK `lj_dg_5_2`**: 알려진 새끼 MCP 0.41rad offset 연장. calib[17] trim 또는 글러브 재캘리브.
- **WARN_TRACKING `lj_dg_3_1`**: 미미함. 무시 OK.

## ▶ 이전 진행 (2026-05-28)

**mock end-to-end 검증 한계 확인:** 사용자 mock 테스트 결과 — reference 토픽엔 정상값, 근데 joint_states pos/vel=0 effort=NaN, 손 안 움직임. 원인: `dg5f_left_mock.launch.py` 가 `dg5f_left_controller` (type=`joint_trajectory_controller/JointTrajectoryController`) 를 spawn 함. 우리 retarget 은 `lj_dg_pospid/reference` (MultiDOFCommand) 로 publish → mock 엔 구독자 없음 → 명령이 컨트롤러까지 못 감. 실물은 `lj_dg_pospid` 가 동봉돼 정상 동작. **mock 은 노드 sanity + reference 흐름까지만 검증 가능, 컨트롤러→HW 단은 mock 으론 못 봄.**

**검증된 것 (mock 으로 충분):** retarget 정상기동 + side 선택(0,1 둘 다 구독) + NaN 가드 동작 + reference 값 흐름.

**다음 단계:**
1. mock 으로 4 패키지 모두 reference 흐름만 빠르게 확인 (side 선택 패치 4노드 다 적용 검증).
2. 손 편 상태에서 `echo .../reference --once` 의 values 0 근처(|v|<~0.3) 인지 — calib offset 발열 재발 방지.
3. 실물(use_mock_hardware:=false) 천천히: plain → grasp_mode → sota 둘.
4. Ctrl-C 시 손 limp 확인 (fork 토크 릴리즈 8faba48 효과).

**손 편 자세 reference 측정 (2026-05-29):** slot 12(|0.36rad≈21°), **slot 17(|0.41rad≈24°)** 만 임계(0.3) 초과, 나머지 18개 다 0 근처.
- slot 12 = lj_dg_4_1 (ring spread/abduction) — flexion 아님, stall/충돌 위험 낮음.
- slot 17 = lj_dg_5_2 (pinky MCP flexion) — 굽힘 첫 관절. 정지자세선 URDF 한계 안(clamp), 다만 글러브로 새끼 추가로 굽히면 한계 근처서 stall 가능성 0 아님.
판정: **실물 진행 OK, 단 새끼손가락 주시.** 모터 윙윙/발열 조짐 즉시 Ctrl-C. stall 확인되면 calib[17] trim (예 1.0→0.7) 또는 글러브 재캘리브(손가락 완전히 펴서). 원인 추정 = 자연자세 새끼 MCP 굽힘 + 글러브 neutral 잔여 offset.

**TODO (낮은 우선):** mock 컨트롤러를 lj_dg_pospid 로 맞춰서 mock end-to-end 가능하게. dg5f_driver(벤더) 수정 필요 → 또 fork 처리 가능. 지금 급한 건 아님.

---

## ▶ 이전 진행 (2026-05-27)

**토크 릴리즈 → fork 방식으로 결정:** 벤더 직접수정 되돌렸다가(once), effort-switch/relax/런치분리 검토 후 최종적으로 **dg_hardware 를 fork 해서 on_deactivate/on_shutdown 에 zero-duty 넣는 방식** 채택(사용자 제안). 가장 깔끔(종료 자동 릴리즈, 레이스 없음).
- effort base 검토 결과: 종료 레이스 안 풀림(원인은 컨트롤러종류 아니라 하드웨어가 종료 시 0 안 보냄), relax 만 쉬워지고 위치추종 재설계 필요 → 기각.
- 현재: `dg/dg_hardware` 에 zero-duty 수정 재적용 + 로컬 커밋 **`1661366`** (vendor 497e7b3 위, main). remote 는 아직 vendor(tesollodelto).
- **✅ fork push 완료:** `LeonJung/dg_hardware` (fork of tesollodelto). 로컬 커밋을 fork/main(vendor 최신 90eb9ce) 위로 rebase → push `90eb9ce..8faba48`. 토크 릴리즈 커밋 `8faba48`. 충돌 없었음.
- **이 머신 dg_hardware remote:** origin=tesollodelto(vendor), fork=LeonJung/dg_hardware.
- **테스트 머신 적용:** `cd <dg_hardware>; git remote set-url origin https://github.com/LeonJung/dg_hardware.git; git fetch origin; git checkout main; git pull` (충돌+버려도되면 `git reset --hard origin/main`) → `colcon build --packages-select delto_hardware` + source. fork 가 vendor 업데이트(90eb9ce 등)도 같이 가져옴.
- 이 머신 빌드 캐시는 타 머신(/home/bpearson) 경로라 컴파일 확인 불가(코드는 write() 기존 멤버만 사용).
- fork 생성은 사용자 웹 Fork(이 머신엔 gh/토큰 없어 API repo 생성 불가, SSH 는 push 전용).

(이하 이전 토크 분석 기록)
**노드 종료 후 손 말려올라감 → 토크 릴리즈 추가:** DG5F 는 토크 enable/disable 레지스터 없이 `SendDuty(duty[])` PWM 방식 → 펌웨어가 마지막 듀티를 계속 물어서 종료 후에도 손 에너자이즈됨. 벤더 `dg_hardware/src/system_interface.cpp` 의 `on_deactivate`+`on_shutdown` 이 Disconnect 전에 duty 0 을 안 보냈음(버그).
- **수정:** 두 훅에서 Disconnect 전에 `SendDuty(zero_duty)` (effort_commands_.size() 만큼 0) 추가. dg_hardware 는 자체 git repo(우리 ss_prep 아님) → 패치파일 `~/hand_ws/torque_release_on_shutdown.patch` 로 사용자에게 전달. 테스트 머신에서 `git apply` + `colcon build --packages-select delto_hardware`.
- **동작 조건 주의:** 컨트롤러 deactivate(=런치 Ctrl-C/컨트롤러 종료) 시에만 발동. teleop 노드만 죽이고 ros2_control_node 살아있으면 컨트롤러가 마지막 reference 유지 → 안 풀림.
- 이 머신선 컴파일 확인 불가(build 캐시가 타 머신 /home/bpearson 경로 참조). 코드는 write() 기존 멤버만 사용.



**상태 업데이트:** 왼손 Manus 글러브 **재보정 완료, 정상 작동** (NaN 없음, plain retarget 동작). 이제 나머지 3패키지(grasp_mode / sota_retarget_a / sota_retargeting_a_good) 가 실제로 도는지 사용자가 재검증 단계.

**확인된 사실:** sota 두 패키지의 `config/left_hand.yaml` 은 left 로 제대로 오버라이드됨(input_topic /manus_glove_0, expected_side left, output_topic /dg5f_left/lj_dg_pospid/reference, joint_states_topic /dg5f_left/joint_states). 노드 기본값은 right 이지만 yaml 이 덮음 → 배선 정상. QoS 도 원인 아님(plain 이 RELIABLE 로 받았음). 글러브 정상화로 NaN 도 해소 → sota 도 이제 돌 것으로 예상.

**사용자에게 준 런북 (재검증용, mock 먼저):**
- 공통 준비: 테스트 머신 `git pull` + `colcon build --packages-select manus_dg5f_retarget manus_dg5f_grasp_mode manus_dg5f_sota_retarget_a manus_dg5f_sota_retargeting_a_good` + source. (NaN 가드 7a8f04d 반영 위해 pull 필수)
- grasp_mode: `ros2 launch manus_dg5f_grasp_mode teleop_with_grasp_mode_left.launch.py use_mock_hardware:=true manus_source:=real default_mode:=free` → echo `reference_free`(retarget) + `reference`(mux) 둘 다 확인. reference_free 만 나오고 reference 비면 grasp_mode_node free-forward 문제(node list/default_mode 확인).
- sota_retarget_a: `ros2 launch manus_dg5f_sota_retarget_a manus_teleop_left.launch.py use_mock_hardware:=true manus_source:=real contact_aware:=false` → 콘솔 solve_ik 예외 + echo reference. (_pad 변형 있음)
- sota_retargeting_a_good: 동일 패턴.
- 공통 검증: `ros2 node list`(크래시?), `ros2 topic echo /manus_glove_0`(값변화+side), `ros2 topic echo /dg5f_left/lj_dg_pospid/reference`, `ros2 topic info -v .../reference`(pub/sub/Reliability), mock 이면 `echo /dg5f_left/joint_states` 로 명령 반영 확인, `ros2 control list_controllers -c /dg5f_left/controller_manager`.

**판정 트리:** glove 빔/side불일치 → 글러브·side필터 / reference_free·reference 빔 → 해당 retarget 발행 안 함(sota=IK실패 콘솔) / reference 나오는데 joint_states 안 변함 → 컨트롤러 spawn·active 문제 / topic info sub=0 → 토픽이름·네임스페이스 불일치.

**grasp_mode 검증 결과 (2026-05-27):** `reference_free` + `reference` 둘 다 빔. `/grasp_mode key_grip` pub 하면 grasp_mode_node 가 "mode=key_grip no glove data yet" 계속 찍음.
→ retarget 도 grasp_mode_node 도 글러브 못 받음 = 공통 입력 `/manus_glove_0` 이 이 세션에선 안 흐름.
→ 단 plain retarget 은 같은 토픽·side필터로 움직였음. **유력 가설: grasp_mode 런치에선 글러브 publisher(manus_data_publisher, manus_ros2)가 안 떠서 /manus_glove_0 빔.** (manus_ros2 는 이 워크스페이스에 없음 → manus_source:=real 이 못 띄움 가능성)
**대기 중인 확인:** grasp_mode 띄운 채 `ros2 topic info /manus_glove_0`(pub count), `ros2 node list|grep manus`, `echo /manus_glove_0 --once`(데이터+side). 그리고 plain 돌렸을 때 글러브를 어떻게 띄웠는지(별도 터미널? 같은 manus_source:=real?).

**★ 원인 확정 (2026-05-27):** `/manus_glove_0` pub=1(manus_data_publisher) sub=2(retarget+grasp_mode), 데이터는 나오는데 **`side: Right` 뿐.** = 왼손 런치가 오른손 글러브를 받음 → 왼손 노드 expected_side=left 가 전 프레임 드랍 → 안 움직임. **코드 아님, 글러브↔토픽 매핑 문제.**
- "왜 plain 은 됐다 안 됐다": Manus 가 글러브를 **연결 순서로 인덱싱** → 세션마다 glove_0/1 뒤바뀜. 아까는 왼손=glove_0, 지금은 오른손=glove_0.
- **확인 대기:** `ros2 topic echo /manus_glove_1 --once` 의 side. Left 면 인덱스 스왑(→MANUS Core 에서 왼손=glove0 으로, 또는 왼손노드 input_topic:=/manus_glove_1). 없으면 왼손 글러브 ROS 스트리밍 안 됨(재페어링).
- 잠재적 영구 개선안: side 필터 대신 **side 기준으로 글러브 선택**(코드/런치에 input_topic 인자화), grasp_mode 런치는 glove_topic 하드코딩이라 인자화 필요.

**★ 확인 완료:** /manus_glove_0=Right, /manus_glove_1=Left (둘 다 스트리밍). = 인덱스 스왑. 왼손 노드가 glove_0(Right) 만 보고 side필터로 다 드랍한 것.

**★ 코드 수정 + push 완료 (커밋 `7fdc529`, origin/main):** 토픽 인덱스 대신 **`side` 필드로 글러브 선택**. 네 노드(retarget, grasp_mode, sota_a, sota_a_good)가 후보 글러브 토픽(/manus_glove_0,_1) **둘 다 구독**하고 기존 side 필터로 자기 손만 취함 → Manus 인덱스 0↔1 스왑돼도 항상 맞는 손. expected_side="any" 면 기존 단일 토픽 유지. 새 param: input_topics / glove_topics.

**다음 (사용자, 테스트 머신):** `git pull` → `colcon build --packages-select manus_dg5f_retarget manus_dg5f_grasp_mode manus_dg5f_sota_retarget_a manus_dg5f_sota_retargeting_a_good` → source → grasp_mode 다시 mock 으로: reference_free/reference 둘 다 값 나오고 손 움직임 기대. 확인되면 sota 둘도, 그 다음 실물.

**✅ 해결 (2026-05-27):** 사용자 pull+build 후 grasp_mode 작동 확인. side 기반 글러브 선택(`7fdc529`)으로 인덱스 스왑 무관하게 왼손 동작.

**남은 것:** sota 둘도 동일하게 동작하는지 확인(예상 OK), 그 다음 실물(use_mock_hardware:=false) 검증. 실물 전 mock 에서 reference 거동 확인 규칙 유지.

**완료:** NaN 가드(retarget+grasp_mode) 커밋 `7a8f04d` origin/main push 완료.

---

## 2026-05-26 — "다른 런치는 핸드 안 움직임" 분석 (= 전부 글러브 NaN 의 다른 얼굴)

추론: plain retarget 이 **기본 RELIABLE QoS** 구독으로도 글러브를 받아 움직였음 → 실 Manus publisher 는 RELIABLE 호환 → **QoS 불일치는 이 현상 원인 아님** (앞 경과2 QoS 의심 다운그레이드). 글러브 수신은 정상, 차이는 노드 설계.

| 런치 | 구조 | NaN 글러브 처리 | 결과 |
|---|---|---|---|
| plain manus_dg5f_retarget | glove 콜백서 산수 후 즉시 publish | 가드 없어 NaN 그대로 발행(이번에 가드 추가) | NaN reference → 발작+발열 |
| sota a / a_good | glove 는 저장만(216-220), 타이머 `_tick`서 ergo_to_q0→`solve_ik`(SLSQP)→publish(283) | NaN→solve_ik 실패(예외/미수렴)→tick이 publish 못 감 | reference 안 나감 → 정지 |
| grasp_mode | plain retarget→reference_free→free 통과 | free면 plain처럼 발작이어야 정상 | 안 움직였으면 mock이었거나/default_mode≠free/글러브 미스트림 |

→ "다른 런치 안 움직임"은 sota 별도 버그 아님. **글러브 NaN 해결 + NaN 가드** 면 정리됨.
검증(mock): 각 런치서 `echo /dg5f_left/lj_dg_pospid/reference` — sota는 빔(+콘솔 IK 트레이스백), grasp_mode(free)는 값 나오되 NaN 섞임.

---

## 치트시트 — 왼손 실물 구동 + contact GUI

공통: `source ~/hand_ws/install/setup.bash` / `ping -c2 169.254.186.73`. 실글러브(`manus_source:=real`)는 외부 `manus_ros2`/MANUS Core 필요(워크스페이스에 없음). 실물 구동 핵심 규칙 = teleop 류는 **`use_mock_hardware:=false`**.

**드라이버만** (delto_ip 기본 169.254.186.73, port 502, fingertip_sensor/ft_broadcaster/io 기본 false):
- `dg5f_driver dg5f_left_pid_all_controller.launch.py` → 컨트롤러 `lj_dg_pospid` (teleop 이 쓰는 스택, 표준)
- `dg5f_driver dg5f_left_pid_controller.launch.py` / `..._effort_controller.launch.py` / `dg5f_left_driver.launch.py`(컨트롤러 `dg5f_left_controller`)
- 직접 명령: `/dg5f_left/lj_dg_pospid/reference` 에 control_msgs/MultiDOFCommand

**풀 teleop** (use_mock_hardware:=false 시 내부에서 pid_all_controller 끌어옴):
- `manus_dg5f_retarget manus_teleop_left.launch.py` — args: manus_source(sim|real|external), use_mock_hardware, thumb_cmc_mode(fixed|coupled|raw_nodes_ik). **활성 retarget**
- `manus_dg5f_grasp_mode teleop_with_grasp_mode_left.launch.py` — 위 + default_mode(free|key_grip). grasp mux + lj_dg_1_1 Kp override
- `manus_dg5f_sota_retarget_a manus_teleop_left{,_pad}.launch.py` — args: manus_source, use_mock_hardware, contact_aware. IK 실험
- `manus_dg5f_sota_retargeting_a_good manus_teleop_left.launch.py` — 튜닝 IK(최신)
- `manus_dg5f_retarget retarget_only.launch.py hand_side:=left` — 드라이버 따로 띄운 경우 노드만
- 예: `ros2 launch manus_dg5f_grasp_mode teleop_with_grasp_mode_left.launch.py use_mock_hardware:=false manus_source:=real default_mode:=key_grip`
- 양손 real 동시 = USB 충돌 → 한쪽 manus_source:=external

**Contact GUI (effort=모터전류 기반)**: contact_monitor 가 `/dg5f_<side>/joint_states` effort 읽어 `/dg5f_<side>/contact_level` 발행, contact_viz(PyQt5) 램프 표시. 인자 없음, config/contact_thresholds_left.yaml 로 설정. 드라이버 먼저 떠 있어야 함.
- `dg5f_contact_viz contact_viz_left.launch.py` (왼손) / `contact_viz_both.launch.py` (양손) / `contact_viz.launch.py` (오른손)

---

## 2026-05-26 — ⚠️ 과열 이벤트: manus_teleop_left 실물에서 손가락 격렬 요동 + 발열

`ros2 launch manus_dg5f_retarget manus_teleop_left.launch.py` (실물, use_mock_hardware:=false 추정) 띄우니 손바닥 편 상태인데도 손가락이 심하게 움직이고 모터 발열 심함 → **Ctrl-C 로 정지 완료**. 모터 stall 전류 = 하드웨어 탈 위험이라 즉시 정지가 정답.

**의심 원인 (점검 중):**
- plain retarget 은 벤더 PID yaml 사용(grasp_mode Kp override 안 거침) → 단순 게인 이슈는 아닐 듯.
- 유력: 왼손 calib/dir_sign 부호 오류 → 손가락을 관절 한계 너머로 밀어 limit 에 stall → 과전류 → 발열. (왼손 calib 은 right 미러, dir_sign 만 flip 구조)
- 또는 flaky 글러브 데이터(QoS/side 건)로 reference 점프.

**재발 방지 규칙:** 실물 켜기 전 항상 mock(use_mock_hardware:=true)으로 거동 먼저 확인. 발열/stall 시 Ctrl-C → 안 되면 핸드 전원 차단.

**경과:** 인자 = `use_mock_hardware:=false manus_source:=real`. 추가로 **다른 런치(grasp_mode/sota)는 손가락 아예 안 움직임**, plain manus_dg5f_retarget 만 격렬.
→ 그림 맞음: plain retarget 만 reference 를 컨트롤러에 **직접** 발행해 유일하게 반응. 나머지는 mux/QoS/side 끊김(위 경과2 건)으로 명령이 컨트롤러까지 안 감.
calib 점검: 왼손 dir_sign 은 right 미러로 대체로 정합(엄지 slot0-3 전부 반전, 각 손가락 spread 만 반전, pinky slot16-17 반전). 명백한 부호 오류는 안 보임. clamp_to_urdf_limits=true (한계 밖 명령은 안 하지만 한계 AT 에서 stall 은 가능).

**다음 (실핸드 금지, mock 으로만):** `manus_teleop_left.launch.py use_mock_hardware:=true manus_source:=real` 띄우고 손 편 상태에서 `ros2 topic echo /dg5f_left/lj_dg_pospid/reference` 값 확인.
- 0 근처면 → 발열은 글러브 튐(QoS flaky)로 인한 reference 점프 → QoS 수정.
- 큰 값/한계 쏠림/점프면 → calib 오프셋·글러브 미보정.
**규칙: echo 로 reference 정상 확인 전까지 use_mock_hardware:=false 재가동 금지.**

**✅ 근본 원인 확정:** `/manus_glove_0` echo 결과 — **왼손 글러브가 NaN 출력** (node id 1~4,6~9 orientation / 1~4,7~9 position 전부 NaN). 오른손은 NaN 없음.
- NaN ergonomics → retarget 공식 → qd_rad NaN. `_clamp`(retarget_node.py:155-156 `lo if v<lo else hi if v>hi else v`)은 NaN 비교가 전부 False라 **NaN 통과** → NaN reference → PID 모터 발작 → stall → 발열. (sota/IK 가 안 움직인 것도 NaN 입력으로 해 못 구해서)
- 두 층 원인: (1) **글러브** — MANUS Core 에서 왼손만 재보정/재페어링/착용·연결 확인 (오른손 정상). (2) **코드** — NaN 가드 없었음.

**수정 (코드, 적용 완료):** 발행 직전 `all(math.isfinite(v))` 가드 추가 — 비정상 프레임 드랍, 컨트롤러는 마지막 정상값 유지.
- manus_dg5f_retarget/retarget_node.py (_on_glove publish 직전)
- manus_dg5f_grasp_mode/grasp_mode_node.py (_publish 진입부, +import math)
- py_compile OK. **커밋 `7a8f04d` origin/main 에 push 완료** (SSH 키 등록 후 c4c8f0f..7a8f04d). 테스트 머신에서 pull + `colcon build --packages-select manus_dg5f_retarget manus_dg5f_grasp_mode` 필요. (follow-up: sota 두 retarget 노드에도 동일 가드 검토)

**다음:** 왼손 글러브 재보정 후 mock 으로 reference 정상(손 폈을 때 0 근처) 확인 → 그 다음에야 실물.

---

## 2026-05-26 — 왼손 grasp-mode teleop 이 실핸드를 안 움직임

**환경 주의:** 사용자는 Claude 가 도는 머신이 아닌 **다른 컴퓨터**에서 테스트 중.
→ 여기서 `ros2 node list` / `ping` 돌려도 무의미. 진단 커맨드는 사용자가 테스트 머신에서 실행해야 함.

**증상:** 아래 커맨드로 띄웠더니 `lj_dg_pospid` configured + activated 까지 다 떴는데 핸드가 안 움직임.
```
ros2 launch manus_dg5f_grasp_mode teleop_with_grasp_mode_left.launch.py
```

**원인 (launch 기본 인자):** `teleop_with_grasp_mode_left.launch.py`
- `use_mock_hardware` 기본 **true** → `dg5f_left_mock.launch.py` 가 올라감. 실 Modbus 드라이버 스택(control_node/rsp/jsb/pid_spawner)은 `UnlessCondition(use_mock_hardware)` 라 안 뜸. → 실핸드(169.254.186.73)엔 명령 안 감. activated 떠도 mock 상대라 의미 없음.
- `manus_source` 기본 **sim** → 실글러브 아니라 시뮬 슬라이더가 입력.
- 최근 튜닝한 lj_dg_1_1 Kp 3.0 override 도 `use_mock_hardware:=false` 일 때만 적용됨.

**해결책 (실핸드 구동):**
```
ros2 launch manus_dg5f_grasp_mode teleop_with_grasp_mode_left.launch.py use_mock_hardware:=false manus_source:=real
```
- 실글러브 없이 슬라이더로 실핸드만 테스트: `manus_source:=sim use_mock_hardware:=false`
- 띄운 뒤 모드 전환(기본 free): `ros2 topic pub --once /grasp_mode std_msgs/String "data: 'key_grip'"`

**사용자 머신에서 돌릴 진단:**
```
ping -c2 169.254.186.73
ros2 control list_hardware_components -c /dg5f_left/controller_manager   # mock vs 실드라이버
ros2 topic echo /dg5f_left/lj_dg_pospid/reference --once                 # 명령 토픽 흐르나
```

**경과 1 (재시도 후):** 사용자 테스트 머신에서 확인 —
- ping 169.254.186.73 OK
- `ros2 control list_hardware_components`: `GD5FLeftHardware`, state active(id=3), 모든 lj_dg_*/effort `available claimed` → **실드라이버 정상 active, mock 아님**. (즉 use_mock_hardware:=false 로 떴음)
- `ros2 topic echo /dg5f_left/lj_dg_pospid/reference --once`: **안 뜸** → 컨트롤러에 목표값 안 들어감.

**원인 재진단:** 하드웨어 OK, 문제는 glove→retarget→grasp_mode→reference 체인이 끊김.
grasp_mode_node 로직(grasp_mode_node.py):
- free/key_grip 둘 다 retarget 의 `reference_free` 가 흘러야 reference 발행됨.
  - free: `_on_ref_in` 받을 때마다 통과(174-175)
  - key_grip: glove 받을 때 발행하나 `_latest_dof_names` 를 첫 reference_free 에서 배워야 함(208-211)
- 따라서 reference 빔 = 그 위 `reference_free` 또는 `/manus_glove_0` 이 안 흐름. 거의 glove 입력 문제.
- 주의: `manus_ros2 manus_data_publisher`(real glove 노드)는 이 워크스페이스 src 엔 없음(manus_ros2_msgs 만 있음) → real 이면 MANUS Core/패키지 별도 필요.

**다음 진단 (사용자 머신):**
```
ros2 node list                                              # retarget/grasp_mode/glove소스 다 떴나
ros2 topic echo /manus_glove_0 --once                       # glove 원본
ros2 topic echo /dg5f_left/lj_dg_pospid/reference_free --once  # retarget 출력
```
+ 띄울 때 manus_source 를 real 로 줬는지 sim 인지 확인 필요.

**경과 2:** manus_source:=real 로 줬었음. 그리고 "아무것도 안 바꿨는데 또 됨" → 간헐적(flaky) 이슈로 판명. 이전에도 안되다 되다 반복.

**근본 원인 (코드 증거 확보):** calib_wizard 커밋 `2af3088`("BEST_EFFORT QoS + side-filter diagnostics", "8s 동안 glove frame 0개" 실패 대응)에서 고친 두 버그가 **calib_wizard 에만 적용되고 실제 teleop 노드엔 안 들어감**:
1. **QoS 불일치**: 실 Manus publisher = BEST_EFFORT. 근데 retarget_node.py:240 / grasp_mode_node.py:120-123 둘 다 `/manus_glove_0` 을 기본 RELIABLE 로 구독 → RELIABLE sub + BEST_EFFORT pub = 호환 안 됨, 조용히 다 드랍.
2. **side 필터 과엄격**: retarget_node.py:253-254 / grasp_mode_node.py:177-180 `if expected!="any" and side!=expected: return` → 드라이버가 side='' 보내면 드랍. (calib_wizard 는 `if side and side!=expected` 로 빈 side 허용 + --accept-any-side.)
→ 둘 다 discovery 타이밍/글러브 스트리밍 시점/side 필드 일관성에 따라 붙다 말다 함 = flaky.

**제안 수정 (calib_wizard 에서 검증됨):** retarget_node + grasp_mode_node 의 /manus_glove_0 구독을 (a) BEST_EFFORT QoS, (b) side 필터 빈 side 허용 으로 변경.

**상태:** 사용자에게 수정 적용 여부 확인 중. (사용자는 다른 머신에서 pull/rebuild 필요)
