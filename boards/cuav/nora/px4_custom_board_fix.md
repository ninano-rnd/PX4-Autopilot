# CUAV Nora 커스텀 보드(NINANO-Zepher / Yellow) PX4 포팅 기록

이 문서는 커스텀 센서보드(ArduPilot 쪽 코드명 `NINANO-Zepher`, `spi.cpp`상 `IMU_VERSION_ZEPHYR == 1` "Yellow" 버전)에
`boards/cuav/nora` 기반 PX4 펌웨어를 포팅하는 과정에서 발견된 문제와 원인, 수정 내역을 정리한다.

## 배경

- 하드웨어: CUAV Nora와 핀 호환되는 커스텀 보드. 부트로더 호환을 위해 `APJ_BOARD_ID AP_HW_CUAV_NORA`를 그대로 사용.
- 센서 구성(Yellow 버전, `boards/cuav/nora/src/spi.cpp`에 이미 반영됨):
  - SPI1: BMI088 (accel + gyro)
  - SPI2: RM3100 (compass) + FRAM
  - SPI4: ICM42688P + MS5611(baro)
  - SPI6: ICM45686 + MS5611(baro)
- DSU(디버그/시리얼/USB 콤보) 커넥터를 리워크 중 제거함 → 표준 SWD/디버그 UART(UART7) 접근 불가.
- 증상: 커스텀 보드에 PX4 앱 펌웨어를 올리면 부트로더 단계(장치관리자에 "Ardupilot" COM으로 잠깐 표시)까지는
  오리지널 보드와 동일하게 동작하지만, 이후 USB 시리얼로 재열거되지 않아 QGC 연결이 안 됨. 같은 하드웨어에
  ArduPilot 펌웨어를 올리면 정상 동작(센서 데이터까지 확인됨).

## 1. IMU 3종 / Compass 방향(rotation) 값 수정

**증상/요청**: `rc.board_sensors`의 IMU·컴퍼스 회전값이 실제 장착 방향과 맞지 않음. 사용자가 이미 검증한
ArduPilot hwdef의 `ROTATION_*` 값을 PX4 쪽에 반영해야 함.

**원인**: PX4와 ArduPilot은 같은 회전 enum 체계(`src/lib/conversion/rotation.h`)를 공유하지만 rc.board_sensors의
`-R` 값은 갱신되지 않은 채였고, 애초에 `rc.board_sensors`가 참조하는 드라이버 구성 자체가 구버전(Original, `IMU_VERSION_ZEPHYR==0`) 하드웨어 기준(icm20689/bmi088+icm42688p 폴백/icm20649+icm20689 폴백)으로 남아 있어 실제 Yellow 하드웨어(spi.cpp에 이미 반영된 bmi088/icm42688p/icm45686 단일 구성)와도 맞지 않았음.

**수정**: [`ROMFS`가 아닌] `boards/cuav/nora/init/rc.board_sensors`
- SPI1: `icm20689` → `bmi088 -A`/`-G` (Yellow 버전은 SPI1에 BMI088이 배선됨)
- SPI4: `bmi088`/`icm42688p` 폴백 로직 제거 → `icm42688p` 단독
- SPI6: `icm20649`/`icm20689` 폴백 로직 제거 → `icm45686` 단독
- 회전값 매핑 (ArduPilot `ROTATION_*` → PX4 rotation enum 숫자):

  | 센서 | 버스 | ArduPilot ROTATION | PX4 `-R` |
  |---|---|---|---|
  | BMI088 (A/G) | SPI1 | `ROTATION_PITCH_180_YAW_90` | 26 |
  | RM3100 (compass) | SPI2 | `ROTATION_ROLL_180_YAW_90` | 10 |
  | ICM42688P | SPI4 | `ROTATION_PITCH_180_YAW_270` | 27 |
  | ICM45686 | SPI6 | `ROTATION_PITCH_180` | 12 |

- 외부 `ist8310`(GPS1/I2C1 컴퍼스 퍽)은 기존 `-R 10`이 hwdef의 (주석 처리된) IST8310 항목과 이미 일치해 변경 없음.

**추가로 필요했던 것**: `icm45686` 드라이버가 `boards/cuav/nora/default.px4board`에 비활성 상태였음
(`CONFIG_DRIVERS_IMU_INVENSENSE_ICM45686`이 없었음) → 추가하지 않으면 `icm45686` 명령 자체가 존재하지 않아
rc.board_sensors가 실패함. `CONFIG_DRIVERS_IMU_INVENSENSE_ICM45686=y` 추가.

## 2. RM3100 Y축 반전 (커넥터 반전 배선 보정)

**증상/요청**: 실제 하드�웨어의 RM3100 Y축 커넥터가 반전 배선되어 있어, ArduPilot 쪽은
`define AP_RM3100_REVERSAL_MASK 2` (bit1 = Y축)로 보정함. PX4에도 동일하게 반영 필요.

**원인**: PX4의 `rm3100` 드라이버(`src/drivers/magnetometer/rm3100/`)에는 축 단위 부호 반전 기능이
아예 없었음. `-R`(rotation)은 정상 회전(determinant +1)만 표현 가능하고, 한 축만 부호를 뒤집는 것은
수학적으로 반사(determinant -1)라 회전값으로는 표현 불가능 — 즉 하드웨어 결함을 보정하려면 rotation과는
별개로 raw 데이터 단계에서 부호를 뒤집는 기능이 필요했음.

**수정**:
- `src/drivers/magnetometer/rm3100/rm3100.h`: `_axis_reversal_mask` 멤버 추가 (bit0=X, bit1=Y, bit2=Z)
- `src/drivers/magnetometer/rm3100/rm3100.cpp`: 생성자에서 `config.custom1`으로 마스크를 받고,
  raw 데이터 범위 검증 후 `_px4_mag.update()` 호출 전에 해당 축 부호를 반전
- `src/drivers/magnetometer/rm3100/rm3100_main.cpp`: 새 CLI 옵션 `-M <mask>` 추가
  (기존 PX4 드라이버들의 `-C`/`custom1` 관례를 따름). 다른 보드의 rm3100 사용에는 영향 없는 옵트인 방식.
- `boards/cuav/nora/init/rc.board_sensors`: `rm3100 -s -b 2 -R 10 -M 2 start` (bit1=2 → Y축만 반전)

## 3. 부팅/USB 미연결 문제 진단 및 근본 원인

### 3.1 디버그 콘솔 확보

DSU 포트 제거로 SWD/UART7 디버그 접근이 불가능한 상태에서 원인 파악을 위해 NuttX 콘솔을 다른 UART로 옮김.

- 1차: UART7 → USART2(TELEM1)로 이동 (`nuttx-config/nsh/defconfig`: `CONFIG_UART7_SERIAL_CONSOLE` 제거,
  `CONFIG_USART2_SERIAL_CONSOLE=y` 추가, 흐름제어 비활성화 — 디버그 어댑터는 보통 RTS/CTS를 안 물리므로
  흐름제어가 걸려 있으면 TX가 CTS 대기로 막힐 수 있음).
- 문제 발견: TELEM1에는 `MAV_0_CONFIG` 기본값(`TEL1`)으로 MAVLink 인스턴스가 자동으로 붙기 때문에,
  콘솔 텍스트와 MAVLink 바이너리 스트림이 같은 UART에서 뒤섞여 로그가 깨지고(화면이 초기화되는 것처럼 보임),
  타이핑한 NSH 명령어조차 깨져서 `command not found`가 남.
- 2차(최종): USART2 설정을 원복하고, 콘솔을 **USART6(TELEM2)**로 이동. `MAV_1_CONFIG`(TEL2) 기본값이
  비어있어(RAM 절약 목적으로 기본 비활성화) 아무 MAVLink 트래픽도 붙지 않는 깨끗한 포트이기 때문.

### 3.2 "리부팅 루프"였는지 확인

USART6 콘솔로 받은 깨끗한 로그에서 `ERROR [heater] heater 1: no sensor_accel instances available`가
정확히 `heater.cpp`의 `ScheduleDelayed(1_s)` 주기와 일치하는 1초 간격으로만 반복됨을 확인.
→ **시스템이 크래시/리부팅 루프에 빠진 게 아니라, 정상적으로 부팅되어 유저스페이스까지 안정적으로
살아있는 상태**임을 확인 (heater 에러 자체는 `rc.board_sensors`에서 센서를 모두 주석 처리해 뒀기 때문에
나는 정상적인 부수 효과일 뿐, 원인이 아님).

이 결론으로 이전에 세웠던 "`board_app_initialize()`/`stm32_spiinitialize()` 단계에서 하드웨어 결함으로
행(hang)이 걸린다"는 가설은 전부 폐기됨.

### 3.3 USB 관련 가설들 (기각)

- **PH15 (`USB_HS_ENABLE`)**: PX4 `board_config.h`에서 `PX4_GPIO_PIN_OFF(GPIO_HS_USB_EN)`로 플로팅
  입력 처리되고 있어 초기에는 유력한 원인으로 의심했으나, **사용자가 실제 회로도를 확인한 결과 PH15는
  NC(미배선)**로 확인되어 기각.
- **PA9 (`GPIO_OTGFS_VBUS`) — NuttX 저수준 USB 디바이스 드라이버 관점**: STM32H7 OTG_FS 드라이버
  (`stm32_otgdev.c`)는 `CONFIG_USBDEV_VBUSSENSING`이 꺼져 있으면 VBUS 핀 상태와 무관하게 세션을
  강제로 valid 처리하도록 컴파일됨. 이 보드 defconfig에는 해당 옵션이 없어 **저수준 USB 열거 자체는
  VBUS 핀과 무관**함을 확인 → 이 경로로는 기각.

### 3.4 결정적 테스트: `sercon` 수동 실행

콘솔에서 `sercon` 명령을 수동으로 실행하자 **`CDC/ACM 드라이버가 정상 등록되고 장치관리자에 COM이
바로 나타남**을 확인. 이는 USB 하드웨어(D+/D-, 커넥터, 풀업)가 완전히 정상임을 의미함 — 즉 문제는
하드웨어가 아니라 **자동시작(autostart) 로직이 실행되지 않는 것**으로 완전히 좁혀짐.

(단, `sercon`만으로는 MAVLink가 안 붙어 QGC에 데이터가 안 보였는데, 이는 `sercon`이 raw 시리얼
드라이버만 등록할 뿐 그 위에 MAVLink 인스턴스를 붙이는 건 별도 단계이기 때문 — 원인이 아니라 수동
테스트의 한계였음.)

### 3.5 진짜 원인: `cdcacm_autostart`의 VBUS 감지 게이트

`ROMFS/px4fmu_common/init.d/rcS`:
```sh
if param greater -s SYS_USB_AUTO -1
then
    if ! cdcacm_autostart start
    then
        sercon
        echo "Starting MAVLink on /dev/ttyACM0"
        mavlink start -d /dev/ttyACM0
    fi
fi
```
`cdcacm_autostart start`는 모듈을 워크큐에 스폰만 하고 즉시 성공을 리턴하므로, 위 `sercon` +
`mavlink start` 폴백은 **`cdcacm_autostart` 자체가 실패로 리턴할 때만** 실행됨 — 그런데
`cdcacm_autostart`는 스폰 자체는 항상 성공하므로 이 폴백은 사실상 죽은 코드나 다름없고, 실제 USB
연결은 전적으로 `cdcacm_autostart` 모듈의 내부 상태머신에 의존함.

`src/drivers/cdcacm_autostart/cdcacm_autostart.cpp`의 상태머신(`state_disconnected()`):
```cpp
_vbus_present = (board_read_VBUS_state() == PX4_OK);
...
if (_vbus_present && _vbus_present_prev) {
    sercon_main(0, nullptr);   // 성공해야 다음 단계(mavlink 시작)로 진행
    ...
}
```
`_vbus_present`가 계속 false면 `sercon`도, 그 뒤의 `mavlink start -d /dev/ttyACM0`도 **영원히 호출되지
않음**. 정확히 관찰된 증상(부트로더 이후 USB가 다시 안 뜸)과 일치.

`board_read_VBUS_state()`의 기본(제네릭) 구현(`platforms/nuttx/src/px4/common/board_ctrl.c`):
```c
#if defined(GPIO_OTGFS_VBUS)
int board_read_VBUS_state(void)
{
    return (px4_arch_gpioread(GPIO_OTGFS_VBUS) ? 0 : 1);
}
#endif
```
`GPIO_OTGFS_VBUS`는 `boards/cuav/nora/src/board_config.h`에서 **PA9**로 정의돼 있는데, 위 3.3에서
확인했듯 **PA9는 이 보드에서 NC**임. 즉 이 핀은 절대 "VBUS present"를 보고할 수 없고,
`cdcacm_autostart`는 영원히 `disconnected` 상태에 머무름 — 이게 최종 근본 원인.

반면 `board_config.h`에는 이미 실제로 배선된 신호가 **다른 이름으로 정확히 정의돼 있었음**:
```c
#define GPIO_nPOWER_IN_C       /* PG0 */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTG|GPIO_PIN0)
#define GPIO_nVDD_USB_VALID    GPIO_nPOWER_IN_C   /* USB is Chosen */
#define BOARD_ADC_USB_VALID    (!px4_arch_gpioread(GPIO_nVDD_USB_VALID))
```
ArduPilot hwdef의 `PG0 VBUS_nVALID INPUT PULLUP`과 정확히 같은 핀·풀업 설정이며, 이미 브릭(전원)
선택 로직에서 정상적으로 사용되고 있던 신호였음. `GPIO_OTGFS_VBUS`(PA9)만 잘못된 핀을 가리키고 있었던 것.

## 4. 최종 수정: `board_read_VBUS_state()`를 실제 배선(PG0)으로 교체

**왜 이렇게 고쳤는지**: `GPIO_OTGFS_VBUS`를 그냥 PG0로 바꿔치기하면 안 되는 이유가 두 가지 있었음.

1. **극성이 반대**: 제네릭 구현은 액티브-하이(핀이 HIGH일 때 VBUS present)를 가정하는데, PG0는
   `nVALID`(액티브-로우, LOW일 때 valid)라 그대로 대입하면 로직이 뒤집힘.
2. **`GPIO_OTGFS_VBUS`는 부트로더 쪽 코드에서도 직접 참조됨**: `boards/cuav/nora/src/hw_config.h`의
   `BOARD_VBUS` 매크로, `boards/cuav/nora/src/bootloader_main.c`의 `stm32_configgpio(GPIO_OTGFS_VBUS)`,
   그리고 공용 부트로더 arch 코드 `platforms/nuttx/src/bootloader/stm/stm32_common/main.c`까지
   이 매크로가 정의돼 있다고 가정하고 컴파일됨. 그냥 지워버리면 부트로더 빌드가 깨짐
   (`'GPIO_OTGFS_VBUS' undeclared` 컴파일 에러로 실제 확인함).
3. **심볼 충돌**: `GPIO_OTGFS_VBUS`를 앱 빌드에서도 그대로 살려두면, `board_ctrl.c`의 제네릭
   `board_read_VBUS_state()`가 `#if defined(GPIO_OTGFS_VBUS)`에 의해 컴파일되고, 여기에 새로
   같은 이름의 함수를 앱 전용 파일에 추가하면 링커가 "multiple definition" 에러를 낸다
   (같은 `.a` 안의 `boardctrl_read_VBUS_state()`가 다른 곳에서 항상 참조되므로 `board_ctrl.c.o`가
   무조건 링크에 끌려 들어옴).

그래서 다음과 같이 **빌드 타깃별로 분리**하는 방식으로 수정함:

- **`boards/cuav/nora/src/board_config.h`**
  - `GPIO_OTGFS_VBUS`(PA9) 정의를 `#if defined(BUILD_BOOTLOADER)`로 감싸 **부트로더 빌드에서만**
    존재하도록 제한 (부트로더 쪽 코드는 그대로 PA9를 씀 — 부트로더 자체의 VBUS 감지는 이번 수정
    범위 밖이라 건드리지 않음). 앱 빌드에서는 이 매크로가 아예 없으므로 `board_ctrl.c`의 제네릭
    `board_read_VBUS_state()`가 컴파일되지 않음 → 심볼 충돌 없음.
  - `PX4_GPIO_INIT_LIST`에서 이제 정의되지 않는 `GPIO_OTGFS_VBUS` 항목 제거(앱 빌드 기준).
  - `BOARD_ADC_USB_CONNECTED`도 잘못된 PA9 대신 `GPIO_nVDD_USB_VALID`(PG0, 올바른 극성)로 수정
    — 이 매크로는 `system_power` ADC 상태 리포팅에도 쓰이므로 겸사겸사 정합성 확보.

- **`boards/cuav/nora/src/usb.c`** (부트로더/앱 공용 파일이지만 `#if !defined(BUILD_BOOTLOADER)`로
  앱 빌드에서만 컴파일되도록 가드)
  ```c
  __EXPORT int board_read_VBUS_state(void)
  {
      /* GPIO_nVDD_USB_VALID (PG0) is active-low: LOW == USB VBUS present */
      return px4_arch_gpioread(GPIO_nVDD_USB_VALID) ? 1 : 0;
  }
  ```
  다른 보드들(`radiolink/PIX6`, `corvon/743v2` 등)도 이미 `src/usb.c`에서 같은 함수를 오버라이드하는
  게 PX4의 표준 패턴이라 이를 그대로 따름.

- **`BUILD_BOOTLOADER` 매크로를 실제로 두 곳에 정의해야 위 조건부 컴파일이 동작함**:
  - `boards/cuav/nora/src/CMakeLists.txt`: 부트로더 브랜치의 `drivers_board` 타깃에
    `target_compile_definitions(drivers_board PRIVATE BUILD_BOOTLOADER)` 추가
    (`bootloader_main.c`/`usb.c` 컴파일에 필요).
  - `platforms/nuttx/src/bootloader/CMakeLists.txt`: 서브디렉토리 추가 전에
    `add_compile_definitions(BUILD_BOOTLOADER)` 추가. 이게 필요했던 이유는 공용 부트로더 arch 코드
    (`platforms/nuttx/src/bootloader/stm/stm32_common/main.c`)가 `boards/cuav/nora/src/CMakeLists.txt`가
    아니라 이 트리를 통해 별도로 컴파일되기 때문 — `drivers_board` 타깃에만 정의해서는 이 파일까지는
    전달되지 않음(실제로 `'GPIO_OTGFS_VBUS' undeclared` 에러로 확인 후 추가함). 이 변경은 다른 어떤
    보드도 `BUILD_BOOTLOADER`를 참조하지 않으므로(레포 전체 검색으로 확인) 다른 보드 빌드에는
    영향이 없는 순수 추가(additive) 변경임.

**검증**: `make cuav_nora_default`, `make cuav_nora_bootloader` 둘 다 링크 에러 없이 빌드 성공 확인함.

## 5. 검증 결과 (부트로더=ArduPilot + 앱=PX4 혼합 구성)

실제 커스텀 보드에서 **부트로더는 ArduPilot 것을 그대로 두고, 앱 펌웨어만 이번에 빌드한 `cuav_nora_default.px4`로
업로드**해서 테스트함. 결과: 전원 인가, 부팅, 시스템 전반 정상 동작 확인. (즉 3~4장에서 찾은 VBUS/`cdcacm_autostart`
원인 분석과 수정이 실제 하드웨어에서도 유효함이 확인됨 — 부트로더 쪽 VBUS 감지는 손대지 않았어도 앱 자체는
정상적으로 USB/시스템이 살아남.)

다만 GCS(Mission Planner, Status 탭)로 확인해보니 **IMU 회전값을 다시 점검해야 할 것으로 보임** — 아래 6절 참고.

## 6. IMU 데이터 확인 — ax/ay/az가 하나만 표시되는 문제

**증상**: Mission Planner Status 탭에서 `ax, ay, az`(값 있음: 약 0.0996 / 0.0283 / 9.8311)만 채워져 있고,
`ax2, ay2, az2` / `ax3, ay3, az3`는 전부 0.

**확인**: Mission Planner Status 탭의 `ax/ay/az`, `ax2/ay2/az2`, `ax3/ay3/az3`는 각각 별개의 MAVLink 메시지
(`RAW_IMU`=IMU1, `SCALED_IMU2`=IMU2, `SCALED_IMU3`=IMU3)를 매핑한 것으로, **센서 값이 하나로 합쳐진 게
아니라 원래부터 3개의 독립된 슬롯**임. Mission Planner Status 탭은 수신되지 않는 필드도 0/NaN으로 계속
표시해두는 방식이라(예: `airspeed_temp = NaN`, `battery_mahperkm = -∞`), 이 화면만으로는
"IMU2/3가 실제로 인식이 안 된 것"인지 "MAVLink 스트림에 SCALED_IMU2/3가 아직 안 실려 나가는 것"인지
구분되지 않음 → **`nsh> listener sensor_accel -a`로 uORB 상에서 실제 몇 개의 `sensor_accel` 인스턴스가
살아있는지 직접 확인 필요** (결과 미확인, 후속 확인 예정).

**로테이션 값 재검토**: 사용자가 GCS로 자세/가속도 데이터를 실제로 확인한 결과, 1절에서 반영한 회전값에
대한 재수정이 필요할 것으로 보임 (구체적으로 어느 축/어느 센서가 어떻게 틀어져 있는지는 아직 특정 전 —
후속 확인 필요).

## 7. 남은 작업 / 확인 필요 사항

- [ ] 부트로더(`cuav_nora_bootloader.px4`)와 앱(`cuav_nora_default.px4`) 둘 다 커스텀 보드에 재플래시
      (부트로더는 SWD/JTAG 필요 — DSU 제거로 USB만으로는 부트로더 갱신 불가할 수 있음).
- [ ] 재부팅 후 자동으로 USB가 CDC-ACM 시리얼로 열거되는지, QGC가 자동 연결/데이터 수신되는지 확인.
- [ ] `rc.board_sensors`에서 주석 처리해 둔 센서 `start` 라인들 복원 후 정상 동작 확인.
- [ ] 디버그용으로 옮긴 콘솔(USART6/TELEM2)은 계속 디버그 용도로 둘지, 필요 시 원래 UART7로 되돌릴지 결정.
      (UART7은 물리적으로 접근 불가하므로 실사용에는 영향 없지만, 다른 목적으로 TELEM2를 쓰고 싶다면
      원복 검토 필요)
- [ ] 부트로더 자체의 VBUS 감지(PA9, NC)는 이번 수정 범위에서 제외함 — 부트로더 동작에 VBUS 감지가
      실제로 필요한지, 필요하다면 부트로더 쪽도 PG0 기준으로 고쳐야 하는지는 별도 확인 필요.
- [ ] `listener sensor_accel -a`로 3개 IMU(BMI088/ICM42688P/ICM45686)가 모두 uORB에 인스턴스로
      떠 있는지 확인 (Mission Planner Status 탭에서는 IMU1만 값이 보이고 IMU2/3는 0).
- [ ] 1절에서 반영한 IMU 회전값(`-R 26`/`27`/`12`, RM3100 `-R 10 -M 2`) 재검토 — GCS 자세/가속도
      표시가 실제 기체 방향과 맞는지 축별로 확인 필요 (어느 센서/축이 문제인지는 아직 미특정).
