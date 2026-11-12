#include "ifs_algorithm.hpp"

#include <std_msgs/msg/color_rgba.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <utility>

namespace
{
// 힘·모멘트 저역통과 필터의 시정수 [s].
// 계수를 고정값으로 두지 않고 샘플 간격에서 매번 계산하므로, 센서 발행 속도가
// 달라져도 이 시정수가 유지된다. 값을 키우면 부드럽지만 느려지고, 줄이면
// 반응이 빠른 대신 잡음이 통과한다.
constexpr double kForceFilterTauSec = 0.060;

// ---- 접촉점 얼리기 (힘이 급격히 줄 때) ----
// 계산 위치는 접촉힘과 잔류힘의 가중평균으로 나온다.
//     계산 위치 = (접촉힘 x 누른자리 + 잔류힘 x 잔류자리) / (접촉힘 + 잔류힘)
// 손을 떼면 접촉힘만 사라지고 잔류힘은 남으므로, 잔류의 지분이 커지면서 위치가
// 잔류 쪽으로 미끄러진다. 그래서 힘이 빠르게 줄고 있는 동안에는 위치 갱신을
// 멈추고 마지막 값을 유지한다. 접촉과 힘은 계속 보고하므로 민감도는 잃지 않는다.
//
// 힘이 늘거나 일정할 때는 개입하지 않는다(부호로 판정).
// kFreezeEnterRate 를 0 이하로 두면 이 기능이 꺼진다.
// 위치 오차는 잔류/(접촉힘+잔류) 에 비례하므로 힘이 크면 얼릴 이유가 없다.
// 이 값보다 센 접촉에서는 감소율과 무관하게 계속 갱신한다. 이 조건이 없으면
// 선을 그을 때 손 힘의 출렁임을 해제로 오인해 위치가 뚝뚝 끊긴다.
constexpr double kFreezeBelowForce = 5.0;   // [N]
constexpr double kFreezeEnterRate = 6.0;    // 이 속도[N/s]보다 빨리 줄면 얼린다
constexpr double kFreezeExitRate = 3.0;     // 이보다 느려지면 푼다 (깜빡임 방지)
// 이 시간 넘게 계속 줄기만 하면 접촉이 끝난 것으로 본다. 갱신을 재개하면
// 안 된다 — 그 시점은 힘이 가장 작아 위치를 제일 못 믿는 구간이다.
constexpr double kFreezeMaxHoldSec = 1.0;

// 어느 면인지 접촉점의 z 로 가른다. JSshape 는 큰 평면이 둘뿐이다.
//   옆면 : 법선 +y,  y = +0.068,           z = -0.013 ~ +0.154
//   윗면 : 법선 -z,  y = -0.062 ~ +0.068,  z = -0.019 ~ -0.013
// 둘 사이는 z = -0.019 ~ -0.013 의 필렛뿐이라 이 값 하나로 갈린다.
constexpr double kSideFaceMinZ = -0.013;  // z 가 이보다 작으면 윗면

// 접촉점 보정 전체 스위치. false 면 옆면·윗면 렌치 배율(apply_FaceGains)과
// 윗면 예전 위치 보정식(correct_ContactPoint)을 모두 건너뛰어, 보정 전 접촉점이
// 나온다. 보정 전후 비교 기록용. 바꾸면 이 패키지를 다시 빌드해야 한다.
constexpr bool kContactCorrection = true;

// 접촉면 후보 조건 (마찰원뿔). 면 법선과 누르는 힘 사이 각이 이보다 크면 그 면은
// 접촉면이 될 수 없다고 본다. 윗면 끝을 위에서 누를 때 작용선이 옆면과 거의 평행해
// 접촉점이 옆면 아래쪽으로 튀던 문제 대응 (10/06). 값 = cos(최대 각).
// cos 70° = 0.34202014. 0.0 으로 두면 예전과 같다 (힘이 면으로 들어가기만 하면 후보).
constexpr float kContactConeMinCos = 0.34202014F;

// ---- 윗면 위치 보정 (계산된 접촉점을 바로 고친다) ----
// 윗면은 힘이 Fz 로 들어와 Fy 가 거의 0 이라, 위의 힘 배율이 먹지 않는다.
// 그래서 접촉점을 계산한 뒤에 직접 고친다. 윗면은 평면이라 면 안에서
// x, y 를 옮겨도 면을 벗어나지 않는다.
//
// 2026-09-15 측정 (윗면 20점, 5x4 격자, 30~40 N) 에서 맞춘 값이다.
//   보정 전       x RMS 4.20 mm (최대 6.63)   y RMS 10.79 mm (최대 18.15)
//   1차만          x RMS 1.69 mm (최대 3.38)   y RMS  4.59 mm (최대 10.16)
//   y 교차항 추가  x 그대로                    y RMS  1.95 mm (최대  3.80)
//
// x 는 1차면 충분하다. y 는 배율이 x 위치에 따라 달라지므로 y 에만
// x 항과 x*y 항을 둔다. 계수를 1.0 / 0.0 으로 두면 해당 항이 꺼진다.
constexpr double kTopFitXScale = 1.06652;     // x_실제 = 배율 * x_계산 + 치우침
constexpr double kTopFitXOffset = -0.001629;  // [m]
// y_실제 = 배율*y + x계수*x + xy계수*x*y + 치우침
constexpr double kTopFitYScale = 1.432817;
constexpr double kTopFitYFromX = -0.062117;       // x 가 y 를 미는 정도 [무차원]
constexpr double kTopFitYFromXY = -1.871220;      // 비틀림 [1/m]
constexpr double kTopFitYOffset = -0.00258794;    // [m]

// 잰 범위(x +-80, y +-45 mm) 밖은 외삽이라 덜 믿을 만하다. 면 밖으로는
// 못 나가게 막아둔다. 값은 메쉬 윗면 평탄부 범위.
constexpr float kTopFitMinX = -0.100F;
constexpr float kTopFitMaxX = 0.100F;
constexpr float kTopFitMinY = -0.062F;
constexpr float kTopFitMaxY = 0.068F;

// ---- 옆면 렌치 보정 ----
// 옆면 접촉점을 위치식으로 직접 옮기지 않고, 접촉점 계산 전에 센서 렌치의
// Fy, Mx, Mz 를 위치별로 보정한다. 배율과 접촉점이 서로 의존하므로 run()에서
// 세 번 반복해 수렴시킨다.
constexpr double kSideGainFyC = 1.05487821;   // Fy 배율 = C + S * x
constexpr double kSideGainFyS = 1.87425565;   // [1/m]
constexpr double kSideGainMxC = 1.54856060;   // Mx 배율 = C + S * x
constexpr double kSideGainMxS = -1.21200167;  // [1/m]
constexpr double kSideGainMzC = 1.03269169;   // Mz 배율 = C + S * z
constexpr double kSideGainMzS = 1.02080201;   // [1/m]
constexpr int kSideGainIterations = 3;

// z 가 높은 구역에서만 세로 배율을 낮춘다. z0 아래는 예전 식과 완전히 같다.
//
// 보정 없이 두면 위로 갈수록 추정이 실제보다 빨리 올라가, 실제 125 mm 를 누르면
// 137 mm 로 뜨고 140 mm 위로는 작용선이 메쉬 위쪽 끝을 지나 접촉점이 아예 안 나왔다.
//
// 2026-09-30 가로 긋기 13줄(z = 95 ~ 154 mm, 5 mm 간격)로 맞춘 값.
//   줄별 중앙값 오차 RMS 10.7 -> 3.0 mm,  z = 140 ~ 150 구간이 되살아남
//   예)  실제 125 -> 137 이던 것이 128 로,  실제 145 -> 실패 이던 것이 144 로
//
// 전용 클램프를 쓰는 이유: kSideGainMaxZ(0.116) 는 Mz 배율(가로 위치)에도 걸려
// 있어서 그 값을 올리면 x 동작까지 바뀐다. 꺾은선 항만 메쉬 끝까지 쓰게 한다.
constexpr double kSideGainMxZ0 = 0.090;      // [m]
constexpr double kSideGainMxZSlope = -3.0;   // [1/m]
constexpr double kSideGainMxZMax = 0.155;    // [m] 메쉬 위쪽 끝

// 배율에 넣는 위치를 이 범위로 가둔다. raw x < -95 mm, raw z > 116 mm인
// 왼쪽 위 모서리에서 보정 지도가 접히므로 그 구역으로 외삽하지 않는다.
constexpr double kSideGainMinX = -0.095;
constexpr double kSideGainMaxX = 0.100;
constexpr double kSideGainMinZ = -0.013;
constexpr double kSideGainMaxZ = 0.116;

// ---- 윗면 렌치 보정 ----
// 옆면과 같은 방식으로, 접촉점을 계산하기 전에 센서 렌치의 Mx, My 를 고친다.
// 윗면은 힘이 거의 +z 로 들어오므로 가로 위치는 My/Fz, 세로 위치는 Mx/Fz 가
// 정한다. 그래서 Mx(세로)와 My(가로)만 고친다. 둘 다 보정 전 값으로 계산한다.
//
//   Mx' = (C + S*x)*Mx + kTopMxFromMy*My + kTopMxFromFz*Fz
//   My' = kTopGainMy*My + kTopMyFromFz*Fz
//
// 2026-09-15 윗면 20점(5x4 격자, 30~40 N) 원본 렌치로 맞춘 값.
//   보정 전 거리 RMS 11.6 mm -> 2.6 mm (최대 4.5), 한 점씩 빼고 맞춘 교차검증 3.1 mm
//   위치 보정식(kTopFit*)과 누를 때 정확도는 같다. 차이는 힘이 비스듬할 때 드러난다.
//
// 각 항의 뜻
//   Mx 배율 C+S*x : 세로가 43% 좁게 읽히고, 그 정도가 가로 위치에 따라 달라짐(비틀림)
//   My 배율       : 가로가 7% 좁게 읽힘
//   My -> Mx      : 면 전체가 기울어 읽힘
//   Fz -> Mx, My  : 위치 전체가 1~2 mm 밀려 읽힘 (센서 원점이 그만큼 어긋난 것과 같다)
//
// false 로 두면 예전 위치 보정식(kTopFit*)을 쓴다. 긋기 비교 실험용.
constexpr bool kTopWrenchCorrection = true;
constexpr double kTopGainMxC = 1.44665259;    // Mx 배율 = C + S * x
constexpr double kTopGainMxS = -1.86018878;   // [1/m]
constexpr double kTopGainMy = 1.06769740;
constexpr double kTopMxFromMy = 0.06746572;
constexpr double kTopMxFromFz = -0.00223556;  // [m]
constexpr double kTopMyFromFz = 0.00130446;   // [m]

// 배율에 넣는 x 를 윗면 끝으로 가둔다. 잰 범위는 계산값 기준 x -76 ~ +78 mm.
constexpr double kTopGainMinX = -0.100;
constexpr double kTopGainMaxX = 0.100;

// ---- 센서2 배율 x 대칭 (2026-10-27) ----
// contact_frame 이 kMirrorGainXFrame 인 노드(센서2)는 x 에 따라 바뀌는 배율 세 개를
// x = 0 기준으로 뒤집어 쓴다:  옆면 Fy, 옆면 Mx, 윗면 Mx.  배율(x) = 센서1 배율(-x).
// 클램프도 뒤집힌 x 에 건다. 꺾은선(옆면 z, 윗면 y), Mz(z), My(상수), 교차항은 그대로다.
//
// 아직 근거는 없다. 상수는 센서1 데이터로만 맞췄고, URDF 상 두 서포터의 x 방향은 같다.
// 센서2 누름 비교(record_points.py --sensor 2)로 확인할 것.
// false 면 두 센서가 같은 식 (10/13 판과 같음).
constexpr bool kMirrorGainX = true;
constexpr const char * kMirrorGainXFrame = "ur10_jsshape_supporter2";

// ---- 센서2 배율 끄기 (2026-10-29) ----
// true 면 contact_frame 이 kNoCorrectionFrame 인 노드(센서2)만 kContactCorrection = false
// 와 같게 돈다: 옆면·윗면 렌치 배율, 윗면 예전 위치식, 모서리 넘어가기 대책을 모두 안 쓰고
// 예전 탐색(작용선이 꿰뚫는 삼각형 + 마찰원뿔)으로 접촉점을 구한다. 센서1 은 그대로.
// 센서2 아랫면·옆면 누름 기록(보정 전 렌치)용. false 면 10/27 판과 같다.
// 10/30 센서2 전용 배율을 넣으면서 false 로 되돌렸다.
constexpr bool kNoCorrectionSensor2 = false;
constexpr const char * kNoCorrectionFrame = "ur10_jsshape_supporter2";

// ---- 센서2 전용 배율 (2026-10-30) ----
// true 면 contact_frame 이 kSensor2GainsFrame 인 노드(센서2)는 아래 kGainsSensor2 를 쓴다
// (센서1 은 kGainsSensor1 = 위의 kSideGain* / kTopGain* 상수 그대로). 이때 kMirrorGainX 는
// 안 건다 — 센서2 자기 좌표로 맞춘 값이라 뒤집으면 안 된다.
// false 면 두 센서가 센서1 상수를 같이 쓰고, 센서2 는 kMirrorGainX 를 따른다 (10/27 판).
// 값과 근거는 kGainsSensor2 정의 옆 주석.
constexpr bool kSensor2OwnGains = true;
constexpr const char * kSensor2GainsFrame = "ur10_jsshape_supporter2";

// ---- 센서2 마찰원뿔 · 모서리 넘어가기 대책 끄기 (2026-11-02) ----
// contact_frame 이 kSensor2GainsFrame 인 노드(센서2)만 해당. 센서1 은 그대로.
// kSensor2NoFrictionCone = true : 센서2 는 마찰원뿔(kContactConeMinCos)을 안 쓴다
//   (예전처럼 힘이 면으로 들어가기만 하면 후보 = cos 0).
// kSensor2NoEdgeTransition = true : 센서2 는 모서리 넘어가기 대책(면 섞기, 꺾은선,
//   가까운 표면점 탐색)을 안 쓴다. 직전 접촉점의 z 로 면 식 하나를 고르고, 작용선이
//   꿰뚫는 삼각형을 찾는다 (고침 1 kPlaneProbeOnMiss 는 그대로).
// 둘 다 false 면 11/02 이전 (10/30 판) 과 같은 동작이다. 단 kGainsSensor2 값은 11/02 것.
constexpr bool kSensor2NoFrictionCone = true;
constexpr bool kSensor2NoEdgeTransition = true;
// kSensor2NoSideGains = true : 센서2 는 옆면 배율(Fy, Mx, Mz)을 안 건다. 아랫면(코드의 윗면)
//   배율은 그대로. 옆면 가로 긋기가 방향(+x / -x)에 따라 z 30 mm 쯤 달라져서 (11/02 실험)
//   마찰 보정항을 맞출 때까지 껐다가, 11/02 오후 My 항을 넣고 다시 켰다 (false).
//   apply_FaceGains 에서만 본다 — 모서리 넘어가기 대책
//   (kSensor2NoEdgeTransition = false) 의 섞기 식에는 안 걸린다.
constexpr bool kSensor2NoSideGains = false;
// kSensor2RawPositionGains = true : 센서2 는 배율을 정할 위치를 "배율 없는 원본 렌치의 작용선과
//   그 작용선이 들어오는 면 평면의 교점" (get_EntryFacePoint) 으로 한 번에 정하고 반복하지 않는다.
//   면도 그 교점으로 정해진다 (힘이 들어가는 면이 둘이면 교점이 더 안쪽인 면).
//   11/02 센서2 배율을 이 방식으로 맞췄다. 옆면 Fy 기울기가 음수라 반복하면 +x 쪽에서
//   x 가 커질수록 배율이 줄어 x 가 더 커지는 되먹임이 있는데, 이 방식엔 그게 없다.
//   교점을 못 구하면 배율 없음. false 면 예전처럼 직전 접촉점에서 반복 (kSideGainIterations).
constexpr bool kSensor2RawPositionGains = true;
// kSensor2NearestSurface = true : 센서2 는 접촉점을 "보정된 작용선에 가장 가까운 표면점" 으로 구한다
//   (모서리 넘어가기 대책의 3) 탐색만, get_NearestSurfacePoint). 작용선이 표면과 만나면 그 교점,
//   커버 밖으로 살짝 비켜 가면 가장 가까운 표면점 (커버 끝 등). 그 거리가
//   kSensor2NearestMaxDist 보다 크면 접촉점 없음. 면 섞기 · 꺾은선은 안 쓴다.
//   false 면 예전 탐색 (작용선이 꿰뚫는 삼각형, get_ContactPoint).
constexpr bool kSensor2NearestSurface = true;
constexpr double kSensor2NearestMaxDist = 0.005;   // [m]
// 옆면 테두리 탐색 (11/04, 센서2): 가장 가까운 표면점이 옆면 테두리 (|x| >= kSensor2RimX 또는
//   z >= kSensor2RimZ, 옆면 쪽 y > kSensor2RimMinY) 에 있으면 거리 한도를 kSensor2NearestMaxDist 대신
//   kSensor2RimMaxDist 로 넓힌다. 커버 끝 (x +-100) 을 긋거나 끝에서 출발할 때 보정된 작용선이
//   커버 밖 7 ~ 14 mm 를 지나 접촉점이 사라지던 것 (11/04 바깥 띠 기록) 을 테두리 점으로 살린다.
//   오프라인 (11/02 저녁 3 배율 그대로, 11/04 긋기, 경계 z <= 20 뺌): 안 나옴 16.6 -> 4.1 %,
//   v100-z 100 -> 42 %, v-100-z 44 -> 2 %, 11/02 긋기 5.2 -> 1.9 %, 누름 그대로 (11/04 바깥 21점 안 나옴 3 -> 0).
//   테두리로 붙은 점: x +-100 세로 획은 x 오차 0, 끝에서 출발하는 가로 획은 세로 오차 5 ~ 16 mm.
//   0 이면 테두리도 kSensor2NearestMaxDist (11/02 판과 같음).
constexpr double kSensor2RimMaxDist = 0.015;   // [m]
constexpr double kSensor2RimX = 0.0995;        // [m] 옆면 가로 끝 (메쉬 x +-100, 끝막이 면 없음)
constexpr double kSensor2RimZ = 0.155;         // [m] 옆면 위쪽 끝 (메쉬 z 최대 155.7)
constexpr double kSensor2RimMinY = 0.050;      // [m] 옆면 쪽만 (아랫면 · 반대쪽은 해당 없음)

// ---- 배율 반복 중 작용선이 커버를 벗어날 때 (10/07) ----
// run() 은 직전 접촉점에서 배율을 정하고 그 배율로 접촉점을 다시 구하기를 되풀이한다.
// 예전에는 도중에 작용선이 커버와 안 만나면 반복을 멈췄다. 그러면 직전 접촉점이
// 갱신되지 않아 다음 틱도 같은 배율로 시작하고, 계속 실패에 갇힌다.
//   - 옆면 z 140 ~ 154 를 왼쪽에서 그어 올 때 앞부분이 통째로 실패
//   - 오른쪽 끝(x 90 ~ 100)을 새로 누를 때 실패
// 그때 "힘이 들어가는 면" 의 무한 평면과 작용선의 교점을 면 범위로 가둬,
// 다음 배율을 정할 위치로만 쓴다. 발행하는 접촉점(cp, 화살표)은 여전히 커버 표면과의
// 교점뿐이다. 마지막까지 표면과 안 만나면 지금처럼 접촉점 없음 (0, 0, 0).
// 다음 틱으로 넘기는 직전 접촉점(cp_)에도 이 점을 남기지 않는다.
// 9/30 가로 긋기 13줄 재계산: z ≥ 140 줄의 접촉점 없는 표본 40% -> 21%.
// false 면 예전과 같다.
constexpr bool kPlaneProbeOnMiss = true;
// 면 평면과 범위 (JSshape_surface.stl 의 평평한 부분, 서포터 좌표계 [m])
constexpr float kSidePlaneY = 0.068F;       // 옆면: 법선 +y
constexpr float kSidePlaneMinZ = -0.013F;
constexpr float kSidePlaneMaxZ = 0.1546F;
constexpr float kTopPlaneZ = -0.019F;       // 윗면: 법선 -z
constexpr float kTopPlaneMinY = -0.0629F;
constexpr float kTopPlaneMaxY = 0.0620F;
constexpr float kFacePlaneMinX = -0.100F;
constexpr float kFacePlaneMaxX = 0.100F;

// 보정된 렌치의 작용선과 "힘이 들어가는 면" 의 무한 평면이 만나는 점을 면 범위로 가둔다.
// 힘이 -y 쪽이 더 크면 옆면, +z 쪽이 더 크면 윗면. 만날 수 없으면 false.
// 배율 반복의 다음 위치로만 쓴다. 접촉점으로 발행하지 않는다.
bool get_FacePlaneProbe(const std::vector<float> & ft, std::vector<float> & out)
{
  const double fx = ft[0];
  const double fy = ft[1];
  const double fz = ft[2];
  const double squared_norm = fx * fx + fy * fy + fz * fz;
  if (squared_norm <= static_cast<double>(std::numeric_limits<float>::epsilon())) {
    return false;
  }
  const double norm = std::sqrt(squared_norm);
  const double ux = fx / norm;
  const double uy = fy / norm;
  const double uz = fz / norm;
  // 작용선이 지나는 점 r0 = (f x tau) / |f|^2  (get_WrenchAxis 와 같은 식)
  const double r0x = (fy * ft[5] - fz * ft[4]) / squared_norm;
  const double r0y = (fz * ft[3] - fx * ft[5]) / squared_norm;
  const double r0z = (fx * ft[4] - fy * ft[3]) / squared_norm;
  const auto clamp = [](double value, float low, float high) {
      return static_cast<float>(
        std::min(std::max(value, static_cast<double>(low)), static_cast<double>(high)));
    };

  out.assign(3, 0.0F);
  if (-uy >= uz) {
    // 옆면을 미는 힘
    if (std::abs(uy) < 1.0e-6) {
      return false;
    }
    const double lambda = (kSidePlaneY - r0y) / uy;
    out[0] = clamp(r0x + lambda * ux, kFacePlaneMinX, kFacePlaneMaxX);
    out[1] = kSidePlaneY;
    out[2] = clamp(r0z + lambda * uz, kSidePlaneMinZ, kSidePlaneMaxZ);
    return true;
  }
  // 윗면을 미는 힘
  if (std::abs(uz) < 1.0e-6) {
    return false;
  }
  const double lambda = (kTopPlaneZ - r0z) / uz;
  out[0] = clamp(r0x + lambda * ux, kFacePlaneMinX, kFacePlaneMaxX);
  out[1] = clamp(r0y + lambda * uy, kTopPlaneMinY, kTopPlaneMaxY);
  out[2] = kTopPlaneZ;
  return true;
}

// 작용선이 실제로 "들어오는" 면의 평면 교점을 면 범위로 가둔다 (kSensor2RawPositionGains 용).
// 힘이 들어가는 면 (옆면: 힘 -y, 윗면: 힘 +z) 이 둘 다면, 볼록한 몸체에서는 작용선이
// 두 평면을 다 지난 뒤에야 몸체 안이므로 교점 매개변수가 큰 (더 안쪽) 면이 들어오는 면이다.
// get_FacePlaneProbe 는 힘 방향 (-y 와 +z 중 큰 쪽) 으로 고르는데, 옆면을 위로 세게 그으면
// 둘이 비슷해져 틱마다 옆면 · 윗면 식이 번갈아 걸렸다 (11/02 v60 아래->위, z 35 <-> 67 mm).
bool get_EntryFacePoint(const std::vector<float> & ft, std::vector<float> & out)
{
  const double fx = ft[0];
  const double fy = ft[1];
  const double fz = ft[2];
  const double squared_norm = fx * fx + fy * fy + fz * fz;
  if (squared_norm <= static_cast<double>(std::numeric_limits<float>::epsilon())) {
    return false;
  }
  const double norm = std::sqrt(squared_norm);
  const double ux = fx / norm;
  const double uy = fy / norm;
  const double uz = fz / norm;
  const double r0x = (fy * ft[5] - fz * ft[4]) / squared_norm;
  const double r0y = (fz * ft[3] - fx * ft[5]) / squared_norm;
  const double r0z = (fx * ft[4] - fy * ft[3]) / squared_norm;
  const auto clamp = [](double value, float low, float high) {
      return static_cast<float>(
        std::min(std::max(value, static_cast<double>(low)), static_cast<double>(high)));
    };

  const bool side_in = uy < -1.0e-6;   // 옆면 (법선 +y) 으로 들어가는 힘
  const bool top_in = uz > 1.0e-6;     // 윗면 (법선 -z) 으로 들어가는 힘
  if (!side_in && !top_in) {
    return false;
  }
  const double t_side = side_in ? (kSidePlaneY - r0y) / uy : 0.0;
  const double t_top = top_in ? (kTopPlaneZ - r0z) / uz : 0.0;
  out.assign(3, 0.0F);
  if (side_in && (!top_in || t_side >= t_top)) {
    out[0] = clamp(r0x + t_side * ux, kFacePlaneMinX, kFacePlaneMaxX);
    out[1] = kSidePlaneY;
    out[2] = clamp(r0z + t_side * uz, kSidePlaneMinZ, kSidePlaneMaxZ);
    return true;
  }
  out[0] = clamp(r0x + t_top * ux, kFacePlaneMinX, kFacePlaneMaxX);
  out[1] = clamp(r0y + t_top * uy, kTopPlaneMinY, kTopPlaneMaxY);
  out[2] = kTopPlaneZ;
  return true;
}

// ---- 모서리 넘어가기 대책 (10/13) ----
// 윗면 → 둥근 모서리 → 옆면으로 넘어갈 때 접촉점이 끊기고(해 없음) 튀던 문제 대응.
// 세 가지를 같이 쓴다. 하나씩만 넣으면 어딘가가 나빠졌다.
//   1) 면 판정: 배율 없이 구한 접촉점(되먹임 없음)의 위치와 힘 방향으로
//      윗면·옆면·모서리 확률을 정하고, 세 식으로 고친 렌치를 그 확률로 섞는다.
//      모서리 몫은 지금 윗면 식(꺾은선 없음)이다. 모서리 가운데는 그 식으로 맞았다.
//      예전에는 직전 접촉점의 z 로 한 식만 골라, 경계에서 두 자리를 왕복했다.
//   2) 꺾은선: 모서리 바로 양옆이 모서리 쪽으로 끌려오던 것을 Mx 배율을 낮춰 되돌린다.
//      윗면은 y > 50 mm, 옆면은 z < 20 mm 에서만 걸린다.
//   3) 탐색: 작용선이 꿰뚫는 삼각형만 찾지 않고, 작용선에 가장 가까운 표면점을 쓴다.
//      마찰원뿔(kContactConeMinCos)은 그대로 적용한다.
// 반대쪽 모서리(배율 없는 접촉점의 y < 0)는 예전 면 식을 쓰고 탐색만 3) 으로 한다.
//
// 10/7 기록 오프라인 재계산 (실물 아님):
//   v 넘어가기      끊김 17.1% -> 0%,  10 mm 넘는 튐 13 -> 0번
//   9/30 옆면 긋기  끊김 28.6% -> 0.2%
//   누름 RMS (10/7 15점 / 9/30 옆면 / 9/15 윗면) 8.4 / 21.8 / 2.6 -> 7.5 / 20.8 / 2.6 mm
//   대가: 모서리 가운데 누름이 윗면 쪽으로 약 2 mm 밀림 (꺾은선 때문)
// false 면 예전과 완전히 같다. kContactCorrection 이 false 면 이것도 꺼진다
// (보정 전 기록이 예전과 같게).
constexpr bool kEdgeTransitionFix = true;
constexpr bool kEdgeTransitionActive = kEdgeTransitionFix && kContactCorrection;
static_assert(
  !kEdgeTransitionActive || kTopWrenchCorrection,
  "kEdgeTransitionFix 는 윗면 렌치 보정(kTopWrenchCorrection = true) 위에서 맞춘 것이다. "
  "윗면 위치 보정식과 비교하려면 kEdgeTransitionFix 도 false 로 둘 것");

// 3) 가까운 표면점. 비용 = 작용선과 삼각형 사이 거리 + kNearestFacePenalty * (1 + n.f).
// 힘이 면을 비스듬히 누를수록(n.f 가 0 에 가까울수록) 불리하다. 가장 싼 삼각형이
// 작용선에서 kNearestMaxDist 보다 멀면 접촉점 없음 (0, 0, 0).
constexpr double kNearestMaxDist = 0.015;      // [m]
constexpr double kNearestFacePenalty = 0.006;  // [m]
// 면 판정용 "배율 없는 접촉점" 은 거리 제한을 사실상 두지 않는다.
constexpr double kNearestRefMaxDist = 1.0;     // [m]

// 1) 면 확률. 단면 좌표 s [m]: 둥근 부분의 윗면 쪽 끝이 0, 옆면 쪽 끝이 kFilletArc.
// 윗면은 음수, 옆면은 kFilletArc 보다 크다.
constexpr double kFilletRadius = 0.006;    // [m] 윗면(z -19)과 옆면(y 68) 사이
constexpr double kFilletCenterY = 0.062;   // [m]
constexpr double kFilletCenterZ = -0.013;  // [m]
constexpr double kFilletArc = kFilletRadius * 1.5707963267948966;  // 반지름 x pi/2
// 위치: 배율 없는 접촉점이 둥근 부분보다 윗면 쪽이면 윗면, 옆면 쪽이면 옆면.
// 방향: 힘과 면 안쪽 법선 사이 각이 kBlendAngleDeg 보다 작으면 그 면을 누르는 힘.
// 둘이 같은 면을 가리키면 그 면, 엇갈리거나 둥근 부분이면 모서리. 경계는 시그모이드로
// 부드럽게 (폭 kBlendLocWidth, kBlendAngleWidthDeg).
// 둥근 부분에서 kBlendGate 넘게 떨어지면 위치만 본다. 평평한 옆면을 그을 때
// 마찰로 기운 힘 때문에 모서리 몫이 섞여 들던 것을 막는다.
// 상수는 시험 전에 정했고 결과에 맞춰 고르지 않았다.
constexpr double kBlendLocWidth = 0.003;   // [m]
constexpr double kBlendAngleDeg = 50.0;
constexpr double kBlendAngleWidthDeg = 8.0;
constexpr double kBlendGate = 0.020;       // [m]

// 2) 꺾은선. 모서리 쪽 방향 위치를 정하는 Mx 배율에 곱한다.
//   윗면: 1 + kTopHingeSlope * max(0, y - kTopHingeY0)      y 62 에서 0.90
//   옆면: 1 + kSideHingeSlope * max(0, kSideHingeZ0 - z)    z -13 에서 0.46
// 10/7 hold 18번 + 9/15 윗면 20점 + 9/30 옆면 24점으로 맞춘 값 (센서1, 센서2 도 같이 씀).
constexpr double kTopHingeY0 = 0.050;      // [m]
constexpr double kTopHingeSlope = -8.0;    // [1/m]
constexpr double kSideHingeZ0 = 0.020;     // [m]
constexpr double kSideHingeSlope = -16.3;  // [1/m]

using Wrench6 = std::array<double, 6>;

// 한 센서가 쓰는 옆면·윗면 렌치 배율 상수 묶음. 식은 get_SideFormula / get_TopFormula.
struct FaceGainSet
{
  // 옆면: Fy' = (FyC + FyS x) Fy,  Mz' = (MzC + MzS z) Mz,
  //       Mx' = (MxC + MxS x)(1 + MxZSlope max(0, z - MxZ0))(1 + HingeSlope max(0, HingeZ0 - z)) Mx
//             + MxFromMy My + MxFromFx Fx + MxFromMz Mz,   Mz' 에 + MzFromMx Mx + MzFromFz Fz + MzFromFx Fx
  //       (마지막 괄호는 모서리 넘어가기 대책의 꺾은선, hinge 일 때만)
  double side_fy_c;
  double side_fy_s;
  double side_mx_c;
  double side_mx_s;
  double side_mz_c;
  double side_mz_s;
  double side_mx_z0;
  double side_mx_z_slope;
  double side_mx_z_max;
  double side_min_x;
  double side_max_x;
  double side_min_z;
  double side_max_z;
  double side_hinge_z0;
  double side_hinge_slope;
  // 옆면 Mx' 에 더하는 My 항 (Mx' = ... + MxFromMy My). 센서2 옆면 가로 긋기 방향 차이 (11/02)
  double side_mx_from_my;
  // 세로 긋기 (마찰 Fz) 방향 차이용 (11/02 저녁, 센서2):
  //   Mx' 에 + MxFromFx Fx,  Mz' = (MzC + MzS z) Mz + MzFromMx Mx + MzFromFz Fz  (모두 원본 값)
  double side_mx_from_fx;
  double side_mz_from_mx;
  double side_mz_from_fz;
  // 옆면 기울어짐 (z 오차가 x 에 비례, 11/02 저녁 3, 센서2): Mx' 에 + MxFromMz Mz (원본 값)
  double side_mx_from_mz;
  // 가로 긋기 (마찰 Fx) 가 x 를 미는 것 (11/04, 센서2): Mz' 에 + MzFromFx Fx (원본 값)
  double side_mz_from_fx;
  // My 채널 섞임 (11/04, 센서2): My' = My + MyFromMx Mx + MyFromMz Mz (원본 값).
  //   센서2 는 법선힘이 만드는 모멘트 (Mx, Mz) 의 일부가 My 로 샌다. 긋기에서 이 My 오차가
  //   마찰력과 곱해져 긋는 방향마다 기울기가 달라지고, 마찰 항으로 평균 차이를 빼면 8자 궤적이 된다.
  double side_my_from_mx;
  double side_my_from_mz;
  // 윗면: Mx' = (MxC + MxS x)(1 + HingeSlope max(0, y - HingeY0)) Mx + MxFromMy My + MxFromFz Fz
  //       My' = My_gain My + MyFromFz Fz
  double top_mx_c;
  double top_mx_s;
  double top_my;
  double top_mx_from_my;
  double top_mx_from_fz;
  double top_my_from_fz;
  double top_min_x;
  double top_max_x;
  double top_hinge_y0;
  double top_hinge_slope;
};

// 센서1 (두 센서 공유였던 값). 위에 흩어진 상수를 그대로 모은 것이다.
constexpr FaceGainSet kGainsSensor1{
  kSideGainFyC, kSideGainFyS, kSideGainMxC, kSideGainMxS, kSideGainMzC, kSideGainMzS,
  kSideGainMxZ0, kSideGainMxZSlope, kSideGainMxZMax,
  kSideGainMinX, kSideGainMaxX, kSideGainMinZ, kSideGainMaxZ,
  kSideHingeZ0, kSideHingeSlope,
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  kTopGainMxC, kTopGainMxS, kTopGainMy, kTopMxFromMy, kTopMxFromFz, kTopMyFromFz,
  kTopGainMinX, kTopGainMaxX,
  kTopHingeY0, kTopHingeSlope};

// 센서2 (2026-11-04, 옆면 M1 = My 섞임 항). 11/02 저녁 3 의 데이터 + 11/04 옆면 바깥 띠 (경계 쪽 z <= 20 mm 뺌)
//   로 옆면 상수를 전부 다시 맞추고 My' = My + 0.191 Mx + 0.102 Mz 를 더했다. 위치는 작용선 계산.
//   오프라인: 같은 줄 두 방향의 '줄 좌표 오차 vs 위치' 기울기 차이 (8자) 20줄 평균 13.8 -> 5.7 mm/100mm
//   (줄 하나씩 빼고 맞춰도 5.9, a 0.18 ~ 0.20, b 0.10 ~ 0.12). 단 x + 끝 세로 (v80, v100) 는 4.7 / 4.4 -> 11.8 / 21.0.
//   노드 재현 (테두리 15 mm): 누름 RMS 10/29 49점 7.1 -> 6.6, 11/02 9점 4.6 -> 5.2, 11/04 바깥 21점 7.8 그대로,
//   11/04 긋기 안 나옴 4.1 -> 2.0 %. 마찰 항 (Mx 의 My 항) 은 0.32 -> 0.13 으로 줄고 Fz 항은 거의 0.
//   맞춤 · 평가: docs/contact_analysis_20261104/my1104.py, my_loo.py, my_m3.py
//   (11/04 V2 = Mz 에 Fx 항, 은 넣었다가 되돌림. side_mz_from_fx 는 0)
// (아래는 11/02 저녁 3 설명. 상수는 11/04 M1 값으로 바뀜)
// 센서2 (2026-11-02 저녁 3). 배율을 끈 채 기록한 센서2 원본 렌치로 맞춘 값.
//   배율 위치는 원본 렌치 작용선이 들어오는 면의 평면 교점 (kSensor2RawPositionGains). 꺾은선 0.
// 옆면: 누름 58점 (side7 49점 10/29 + 3x3 9점 11/02) + 양방향 긋기 (11/02 16:44 가로 10획,
//   17:20 가로 6 · 세로 6획) 를 같이 맞춤. 위치는 노드와 같은 작용선 계산 (r0 = f x tau / |f|^2).
//   Fy 가 -x 쪽에서 크게, +x 쪽에서 작게 읽힌다 -> Fy 기울기 (센서1 과 반대 방향).
//   기울어짐 항 Mx += -0.165 Mz: 이게 없으면 z 오차가 x 에 비례 (-x 끝 +13 ~ +22, +x 끝 -13 ~ -30 mm).
//   마찰 항: 가로로 그으면 My ~ z Fx (Mx 에 +0.324 My, +0.0024 Fx), 세로로 그으면 Fz 가
//   마찰 (Mz 에 -0.0131 Mx, +0.0134 Fz). 없으면 긋는 방향에 따라 자리가 10 ~ 35 mm 달라졌다.
//   센서 내부 간섭인지는 모름 (My 항은 아랫면 누름에 걸면 망가진다). 옆면 전용.
//   오프라인 (작용선, 평면 근사): 누름 RMS 10/29 49점 7.3 mm (최대 16), 11/02 9점 4.4 mm.
//   17:20 기록을 빼고 맞춰도 기울어짐 항 -0.169, 뺀 9점 5.0 mm.
//   긋기 방향 차이: 가로 대부분 10 mm 이내, 세로 8 mm 이내 (세로는 x 가 +1 ~ +9 mm 치우침).
//   맞춤 · 평가: docs/contact_analysis_20261102/fit_tilt.py, tilt_holdout.py
// 윗면(아랫면): 센서2 는 원래 정확해서 배율이 거의 1. 10/30 값 그대로 (누름 28점, top7).
//   누름 오차 RMS 6.6 -> 5.1 mm (참 위치에서 배율을 정했을 때).
constexpr FaceGainSet kGainsSensor2{
  1.0, -3.35370096, 0.92771199, 0.0, 0.87499612, 0.40668068,
  0.090, 0.0, 0.155,
  -0.095, 0.080, -0.013, 0.116,
  0.020, 0.0,
  0.13162633, 0.00241423, -0.02784140, -0.00438111, -0.21072500, 0.0,
  0.19103774, 0.10193178,
  1.05866450, 0.49700059, 1.00339032, -0.02913699, -0.00309520, 0.00108323,
  -0.100, 0.100,
  kTopHingeY0, 0.0};

const FaceGainSet & get_GainSet(bool sensor2)
{
  return sensor2 ? kGainsSensor2 : kGainsSensor1;
}

bool is_ZeroPoint(const std::vector<float> & point)
{
  return point[0] == 0.0F && point[1] == 0.0F && point[2] == 0.0F;
}

// 윗면 식으로 고친 렌치 (Mx, My 만 바뀐다). hinge 면 Mx 배율에 윗면 끝 꺾은선을 곱한다.
// point 의 z 는 보지 않는다. 어느 면 식을 쓸지는 부르는 쪽이 정한다.
// x_sign 이 -1 이면 Mx 배율을 x = 0 기준으로 뒤집는다 (kMirrorGainX).
// g 는 그 센서의 상수 묶음 (kGainsSensor1 / kGainsSensor2).
Wrench6 get_TopFormula(
  const std::vector<float> & raw, const std::vector<float> & point, bool hinge,
  double x_sign, const FaceGainSet & g)
{
  const double x = std::min(
    std::max(x_sign * static_cast<double>(point[0]), g.top_min_x), g.top_max_x);
  const double hinge_gain = hinge ?
    1.0 + g.top_hinge_slope * std::max(0.0, static_cast<double>(point[1]) - g.top_hinge_y0) :
    1.0;
  const double fz = raw[2];
  const double mx = raw[3];
  const double my = raw[4];
  Wrench6 out {raw[0], raw[1], raw[2], raw[3], raw[4], raw[5]};
  out[3] = (g.top_mx_c + g.top_mx_s * x) * hinge_gain * mx + g.top_mx_from_my * my +
    g.top_mx_from_fz * fz;
  out[4] = g.top_my * my + g.top_my_from_fz * fz;
  return out;
}

// 옆면 식으로 고친 렌치 (Fy, Mx, My, Mz 가 바뀐다. My 는 센서2 만, 센서1 은 0 이라 그대로). hinge 면 Mx 배율에 옆면 아래 꺾은선을 곱한다.
// x_sign 이 -1 이면 Fy, Mx 배율을 x = 0 기준으로 뒤집는다 (kMirrorGainX).
// g 는 그 센서의 상수 묶음 (kGainsSensor1 / kGainsSensor2).
Wrench6 get_SideFormula(
  const std::vector<float> & raw, const std::vector<float> & point, bool hinge,
  double x_sign, const FaceGainSet & g)
{
  const double x = std::min(
    std::max(x_sign * static_cast<double>(point[0]), g.side_min_x), g.side_max_x);
  const double z = std::min(
    std::max(static_cast<double>(point[2]), g.side_min_z), g.side_max_z);
  // 꺾은선 항은 전용 클램프를 쓴다 (z 가 side_max_z 에서 잘리면 안 걸린다).
  const double zh = std::min(
    std::max(static_cast<double>(point[2]), g.side_min_z), g.side_mx_z_max);
  const double hinge_gain = hinge ?
    1.0 + g.side_hinge_slope * std::max(
      0.0, g.side_hinge_z0 - std::max(static_cast<double>(point[2]), g.side_min_z)) :
    1.0;
  Wrench6 out {raw[0], raw[1], raw[2], raw[3], raw[4], raw[5]};
  out[1] = raw[1] * (g.side_fy_c + g.side_fy_s * x);
  out[3] = raw[3] * (g.side_mx_c + g.side_mx_s * x) *
    (1.0 + g.side_mx_z_slope * std::max(0.0, zh - g.side_mx_z0)) * hinge_gain +
    g.side_mx_from_my * raw[4] + g.side_mx_from_fx * raw[0] + g.side_mx_from_mz * raw[5];
  out[4] = raw[4] + g.side_my_from_mx * raw[3] + g.side_my_from_mz * raw[5];
  out[5] = raw[5] * (g.side_mz_c + g.side_mz_s * z) + g.side_mz_from_mx * raw[3] +
    g.side_mz_from_fz * raw[2] + g.side_mz_from_fx * raw[0];
  return out;
}

double get_Sigmoid(double value)
{
  return 1.0 / (1.0 + std::exp(-value));
}

// 단면 좌표 s [m]. 윗면은 y - 0.062 (음수), 둥근 부분은 호 길이 0 ~ kFilletArc,
// 옆면은 kFilletArc + (z + 0.013).
double get_EdgeS(const std::vector<float> & point)
{
  const double y = point[1];
  const double z = point[2];
  if (y <= kFilletCenterY) {
    return y - kFilletCenterY;
  }
  if (z >= kFilletCenterZ) {
    return kFilletArc + (z - kFilletCenterZ);
  }
  return kFilletRadius * std::atan2(y - kFilletCenterY, kFilletCenterZ - z);
}

// 1) 윗면·옆면·모서리 확률. raw 는 배율 전 렌치, ref 는 그 렌치로 구한 접촉점(0 이 아님).
FaceWeights get_FaceWeights(const std::vector<float> & raw, const std::vector<float> & ref)
{
  const double fx = raw[0];
  const double fy = raw[1];
  const double fz = raw[2];
  const double norm = std::sqrt(fx * fx + fy * fy + fz * fz);
  const double s = get_EdgeS(ref);
  const double loc_top = get_Sigmoid(-s / kBlendLocWidth);
  const double loc_side = get_Sigmoid((s - kFilletArc) / kBlendLocWidth);
  // 윗면을 누르는 힘은 +z, 옆면을 누르는 힘은 -y 쪽이다.
  constexpr double rad_to_deg = 57.29577951308232;
  const double angle_top = std::acos(std::min(std::max(fz / norm, -1.0), 1.0)) * rad_to_deg;
  const double angle_side = std::acos(std::min(std::max(-fy / norm, -1.0), 1.0)) * rad_to_deg;
  double dir_top = get_Sigmoid((kBlendAngleDeg - angle_top) / kBlendAngleWidthDeg);
  double dir_side = get_Sigmoid((kBlendAngleDeg - angle_side) / kBlendAngleWidthDeg);
  // 둥근 부분에서 kBlendGate 넘게 떨어지면 방향 조건을 풀어 위치만으로 정한다.
  const double far_top = get_Sigmoid((-kBlendGate - s) / kBlendLocWidth);
  const double far_side = get_Sigmoid((s - kFilletArc - kBlendGate) / kBlendLocWidth);
  dir_top += (1.0 - dir_top) * far_top;
  dir_side += (1.0 - dir_side) * far_side;
  const double top = loc_top * dir_top;
  const double side = loc_side * dir_side;
  const double edge = std::max(0.0, 1.0 - top - side);
  const double total = top + side + edge;
  return {top / total, side / total, edge / total};
}

// 1) + 2) 세 식으로 고친 렌치를 확률로 섞는다. 모서리 몫 = 윗면 식(꺾은선 없음).
void apply_BlendedGains(
  const std::vector<float> & raw, const std::vector<float> & point,
  const FaceWeights & weights, double x_sign, const FaceGainSet & g, std::vector<float> & out)
{
  const Wrench6 top = get_TopFormula(raw, point, true, x_sign, g);
  const Wrench6 side = get_SideFormula(raw, point, true, x_sign, g);
  const Wrench6 edge = get_TopFormula(raw, point, false, x_sign, g);
  out.resize(6);
  for (std::size_t i = 0; i < 6; ++i) {
    out[i] = static_cast<float>(
      weights.top * top[i] + weights.side * side[i] + weights.edge * edge[i]);
  }
}

// 3) 의 기하 계산. 이 패키지는 최적화 없이 빌드되어 Eigen 이 아주 느리므로
// (한 틱 약 12 ms) 3차원 벡터를 직접 계산한다.
using Vec3 = std::array<double, 3>;

double dot3(const Vec3 & a, const Vec3 & b)
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

Vec3 sub3(const Vec3 & a, const Vec3 & b)
{
  return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

// a + s * b
Vec3 add_scaled3(const Vec3 & a, double s, const Vec3 & b)
{
  return {a[0] + s * b[0], a[1] + s * b[1], a[2] + s * b[2]};
}

// 3) 작용선 (r0 + t f, f 는 단위벡터) 과 삼각형 (v0, v1, v2) 사이의 최근접 거리.
// closest 에 삼각형 위의 그 점을 넣는다. 꿰뚫으면 거리 0 과 교점, 아니면 세 변 중
// 가장 가까운 점이다.
double get_LineTriangleDistance(
  const Vec3 & r0, const Vec3 & f, const Vec3 & v0, const Vec3 & v1, const Vec3 & v2,
  const Vec3 & normal, Vec3 & closest)
{
  const double n_dot_f = dot3(normal, f);
  if (std::abs(n_dot_f) > 1.0e-9) {
    const Vec3 p = add_scaled3(r0, dot3(sub3(v0, r0), normal) / n_dot_f, f);
    const Vec3 e0 = sub3(v1, v0);
    const Vec3 e1 = sub3(v2, v0);
    const Vec3 vp = sub3(p, v0);
    const double d00 = dot3(e0, e0);
    const double d01 = dot3(e0, e1);
    const double d11 = dot3(e1, e1);
    const double d20 = dot3(vp, e0);
    const double d21 = dot3(vp, e1);
    const double denominator = d00 * d11 - d01 * d01;
    const double v = (d11 * d20 - d01 * d21) / denominator;
    const double w = (d00 * d21 - d01 * d20) / denominator;
    const double u = 1.0 - v - w;
    constexpr double tolerance = 1.0e-9;
    if (u >= -tolerance && v >= -tolerance && w >= -tolerance) {
      closest = p;
      return 0.0;
    }
  }

  double best = std::numeric_limits<double>::infinity();
  const std::array<const Vec3 *, 4> corners {&v0, &v1, &v2, &v0};
  for (std::size_t k = 0; k < 3; ++k) {
    // 변 위의 점 a + s e (0 <= s <= 1) 중 작용선에 가장 가까운 점
    const Vec3 & a = *corners[k];
    const Vec3 e = sub3(*corners[k + 1], a);
    const Vec3 w0 = sub3(a, r0);
    const double e_dot_f = dot3(e, f);
    const double denominator = dot3(e, e) - e_dot_f * e_dot_f;
    double s = denominator > 1.0e-12 ?
      (e_dot_f * dot3(w0, f) - dot3(e, w0)) / denominator : 0.0;
    s = std::min(std::max(s, 0.0), 1.0);
    const Vec3 q = add_scaled3(a, s, e);
    const Vec3 off_line = sub3(q, add_scaled3(r0, dot3(sub3(q, r0), f), f));
    const double distance = std::sqrt(dot3(off_line, off_line));
    if (distance < best) {
      best = distance;
      closest = q;
    }
  }
  return best;
}

// 접촉점을 윗면 또는 옆면에 맞는 식으로 보정한다.
void correct_ContactPoint(
  std::vector<float> & point, const std::vector<float> & force)
{
  (void)force;
  if (!kContactCorrection) {
    return;
  }
  if (point[0] == 0.0F && point[1] == 0.0F && point[2] == 0.0F) {
    return;
  }

  if (point[2] >= kSideFaceMinZ || kTopWrenchCorrection) {
    // 옆면, 그리고 kTopWrenchCorrection 이 켜진 윗면은 apply_FaceGains()에서
    // 렌치를 보정했으므로 위치를 다시 옮기지 않는다.
    return;
  }

  // 윗면 — 가로 x, 세로 y.
  // 보정 전 좌표끼리 곱해야 하므로 x 를 먼저 덮어쓰지 않는다.
  const double raw_x = point[0];
  const double raw_y = point[1];
  const double x = kTopFitXScale * raw_x + kTopFitXOffset;
  const double y = kTopFitYScale * raw_y + kTopFitYFromX * raw_x +
    kTopFitYFromXY * raw_x * raw_y + kTopFitYOffset;
  point[0] = std::min(std::max(static_cast<float>(x), kTopFitMinX), kTopFitMaxX);
  point[1] = std::min(std::max(static_cast<float>(y), kTopFitMinY), kTopFitMaxY);
}
}  // namespace

IfsAlgorithm::IfsAlgorithm(
  rclcpp::Node::SharedPtr node,
  const std::string & contact_sensor_topic)
: node_(std::move(node)),
  ft_value_(6, 0.0F),
  cp_(3, 0.0F),
  cp_temp_(6, 0.0F),
  ft_corrected_(6, 0.0F)
{
  node_->get_parameter_or("use_wrench_tf", use_wrench_tf_, false);
  node_->get_parameter_or("contact_frame", contact_frame_, std::string("base_link"));
  node_->get_parameter_or("wrench_source_frame", wrench_source_frame_, std::string(""));
  node_->get_parameter_or("wrench_tf_timeout_sec", wrench_tf_timeout_sec_, 0.05);
  sensor2_gains_ = kSensor2OwnGains && contact_frame_ == kSensor2GainsFrame;
  // 센서2 전용 배율은 센서2 자기 좌표로 맞춘 값이라 x 대칭을 걸지 않는다.
  gain_x_sign_ =
    (kMirrorGainX && !sensor2_gains_ && contact_frame_ == kMirrorGainXFrame) ? -1.0 : 1.0;
  contact_correction_ =
    kContactCorrection && !(kNoCorrectionSensor2 && contact_frame_ == kNoCorrectionFrame);
  const bool is_sensor2 = contact_frame_ == kSensor2GainsFrame;
  edge_transition_active_ = kEdgeTransitionActive && contact_correction_ &&
    !(kSensor2NoEdgeTransition && is_sensor2);
  contact_cone_min_cos_ = (kSensor2NoFrictionCone && is_sensor2) ? 0.0F : kContactConeMinCos;
  side_gains_off_ = kSensor2NoSideGains && is_sensor2;
  raw_position_gains_ = kSensor2RawPositionGains && is_sensor2 && !edge_transition_active_;
  nearest_max_dist_ =
    (kSensor2NearestSurface && is_sensor2 && !edge_transition_active_) ? kSensor2NearestMaxDist : 0.0;
  rim_max_dist_ = nearest_max_dist_ > 0.0 ? std::max(kSensor2RimMaxDist, nearest_max_dist_) : 0.0;

  if (use_wrench_tf_) {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, node_, true);
  }

  cp_pub_ = node_->create_publisher<geometry_msgs::msg::PointStamped>("/contactsensing/cp", 1000);
  arrow_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
    "/contactsensing/force_arrow", 1000);
  ftsensor_sub_ = node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
    contact_sensor_topic, 100,
    std::bind(&IfsAlgorithm::ftsensor_cb, this, std::placeholders::_1));
  pointcloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud>("/mesh_pc", 100);
  force_magnitude_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
    "/contactsensing/force_magnitude", 100);
  // 보정이 끝난 6축을 그대로 내보낸다. 크기만 담는 force_magnitude 와 달리
  // 방향과 모멘트가 남아 있어 순응 제어에서 쓸 수 있다. 화살표 마커와 같은
  // 값이지만 마커는 시각화용이라 길이 배율이 섞여 있어 제어가 쓰면 안 된다.
  corrected_wrench_pub_ = node_->create_publisher<geometry_msgs::msg::WrenchStamped>(
    "/contactsensing/wrench_corrected", 100);
  // cp 가 유효한 좌표인지 알린다. 힘 임계값이 아니라 cp 기준이다.
  contact_state_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
    "/contactsensing/contact_state", 100);

  node_->get_parameter_or("filename", filename_, std::string("contact_path"));
  node_->get_parameter_or("contact_force_threshold", contact_force_threshold_, 1.0);
  cp_pubdata_.header.frame_id = contact_frame_;

  RCLCPP_INFO(
    node_->get_logger(),
    "Wrench coordinates: TF=%s, source=%s, target=%s, timeout=%.3f s",
    use_wrench_tf_ ? "enabled" : "disabled",
    wrench_source_frame_.empty() ? "<message header>" : wrench_source_frame_.c_str(),
    contact_frame_.c_str(), wrench_tf_timeout_sec_);
  // 어느 바이너리가 도는지 로그로 확인할 수 있게 남긴다.
  RCLCPP_INFO(
    node_->get_logger(),
    "Contact correction (all wrench gains): %s",
    contact_correction_ ? "enabled" : "disabled");
  RCLCPP_INFO(
    node_->get_logger(),
    "Edge transition fix (face blend + hinges + nearest surface point): %s",
    edge_transition_active_ ? "enabled" : "disabled");
  RCLCPP_INFO(
    node_->get_logger(),
    "Gain x mirror (side Fy, side Mx, top Mx about x = 0): %s",
    gain_x_sign_ < 0.0 ? "enabled" : "disabled");
  RCLCPP_INFO(
    node_->get_logger(),
    "Gain set: %s",
    sensor2_gains_ ? "sensor2 own (2026-11-04 side M1: + My crosstalk terms, friction + tilt, no hinges)" : "sensor1 (shared)");
  RCLCPP_INFO(
    node_->get_logger(),
    "Friction cone: %s (min cos %.4f)",
    contact_cone_min_cos_ > 0.0F ? "enabled" : "disabled",
    static_cast<double>(contact_cone_min_cos_));
  RCLCPP_INFO(
    node_->get_logger(),
    "Side gains (Fy, Mx, Mz): %s",
    side_gains_off_ ? "disabled" : "enabled");
  RCLCPP_INFO(
    node_->get_logger(),
    "Gain position: %s",
    raw_position_gains_ ? "raw entry-face plane point, no iteration" :
    (edge_transition_active_ ? "edge transition (nearest surface point)" :
    "previous contact point, iterated"));
  RCLCPP_INFO(
    node_->get_logger(),
    "Contact search: %s (max %.1f mm, side rim %.1f mm)",
    edge_transition_active_ ? "nearest surface point (edge transition)" :
    (nearest_max_dist_ > 0.0 ? "nearest surface point" : "line-triangle intersection"),
    1000.0 * (edge_transition_active_ ? kNearestMaxDist : nearest_max_dist_),
    1000.0 * (edge_transition_active_ ? kNearestMaxDist : rim_max_dist_));
}

void IfsAlgorithm::ftsensor_cb(const geometry_msgs::msg::WrenchStamped::ConstSharedPtr message)
{
  if (ft_measure_flag_) {
    return;
  }

  geometry_msgs::msg::WrenchStamped wrench_in = *message;
  if (!wrench_source_frame_.empty()) {
    wrench_in.header.frame_id = wrench_source_frame_;
  }

  geometry_msgs::msg::Wrench transformed_wrench = wrench_in.wrench;
  if (use_wrench_tf_) {
    if (wrench_in.header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "Cannot transform wrench: message frame is empty and wrench_source_frame is not set");
      return;
    }

    try {
      const auto transform = tf_buffer_->lookupTransform(
        contact_frame_, wrench_in.header.frame_id,
        tf2_ros::fromMsg(wrench_in.header.stamp),
        tf2::durationFromSec(wrench_tf_timeout_sec_));
      tf2::doTransform(wrench_in.wrench, transformed_wrench, transform);
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "Cannot transform wrench from '%s' to '%s': %s",
        wrench_in.header.frame_id.c_str(), contact_frame_.c_str(), exception.what());
      return;
    }
  }

  constexpr double algorithm_sign = 1.0;
  const auto & wrench = transformed_wrench;

  // 샘플 간격으로 계수를 만들어, 센서 발행 속도가 바뀌어도 시정수가 유지된다.
  // 고정 계수를 쓰면 속도가 달라질 때 필터 빠르기가 조용히 바뀐다.
  const double now_sec = node_->now().seconds();
  double dt = (last_sample_sec_ < 0.0) ? 0.0 : (now_sec - last_sample_sec_);
  last_sample_sec_ = now_sec;
  dt = std::min(std::max(dt, 1.0e-5), 0.1);
  const double lambda = 0.05;
// dt / (kForceFilterTauSec + dt)
  ft_value_[0] = lambda * algorithm_sign * wrench.force.x + (1.0 - lambda) * ft_value_[0];
  ft_value_[1] = lambda * algorithm_sign * wrench.force.y + (1.0 - lambda) * ft_value_[1];
  ft_value_[2] = lambda * algorithm_sign * wrench.force.z + (1.0 - lambda) * ft_value_[2];
  ft_value_[3] = lambda * algorithm_sign * wrench.torque.x + (1.0 - lambda) * ft_value_[3];
  ft_value_[4] = lambda * algorithm_sign * wrench.torque.y + (1.0 - lambda) * ft_value_[4];
  ft_value_[5] = lambda * algorithm_sign * wrench.torque.z + (1.0 - lambda) * ft_value_[5];
  force_magnitude_ = std::sqrt(
    ft_value_[0] * ft_value_[0] +
    ft_value_[1] * ft_value_[1] +
    ft_value_[2] * ft_value_[2]);
  ft_measure_flag_ = true;
}

int IfsAlgorithm::get_ClosestSurface(WrenchAxis wrench_axis)
{
  double min_value = std::numeric_limits<double>::infinity();
  int closest_index = 0;

  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  for (int i = 0; i < surface_shape_.get_Nvertex(); ++i) {
    surface_shape_.get_Normal(i, x, y, z);
    if ((x * wrench_axis.f.at[0] + y * wrench_axis.f.at[1] + z * wrench_axis.f.at[2]) >
      -contact_cone_min_cos_)
    {
      continue;
    }

    surface_shape_.get_CenterPoint(i, x, y, z);
    const double distance = get_DistanceLinePoint(wrench_axis, x, y, z);
    if (distance < min_value) {
      min_value = distance;
      closest_index = i;
    }
  }
  return closest_index;
}

float IfsAlgorithm::get_DistanceLinePoint(
  WrenchAxis wrench_axis, float x, float y, float z)
{
  const float force_norm = std::sqrt(
    wrench_axis.f.at[0] * wrench_axis.f.at[0] +
    wrench_axis.f.at[1] * wrench_axis.f.at[1] +
    wrench_axis.f.at[2] * wrench_axis.f.at[2]);
  if (force_norm <= std::numeric_limits<float>::epsilon()) {
    return std::numeric_limits<float>::infinity();
  }

  const std::array<float, 3> point = {
    x - wrench_axis.r0.at[0],
    y - wrench_axis.r0.at[1],
    z - wrench_axis.r0.at[2]};
  const std::array<float, 3> cross = {
    wrench_axis.f.at[1] * point[2] - wrench_axis.f.at[2] * point[1],
    wrench_axis.f.at[2] * point[0] - wrench_axis.f.at[0] * point[2],
    wrench_axis.f.at[0] * point[1] - wrench_axis.f.at[1] * point[0]};

  return std::sqrt(
    cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]) / force_norm;
}

bool IfsAlgorithm::set_ContactShape(
  const std::string & filename, Transformation transform)
{
  surface_shape_.STLImport(filename, transform);
  if (surface_shape_.get_Npoint() <= 0) {
    RCLCPP_ERROR(node_->get_logger(), "STL loader could not read: %s", filename.c_str());
    return false;
  }

  pc_data_.header.frame_id = contact_frame_;
  pc_data_.points.reserve(static_cast<std::size_t>(surface_shape_.get_Npoint()));
  for (int i = 0; i < surface_shape_.get_Npoint(); ++i) {
    geometry_msgs::msg::Point32 point;
    surface_shape_.get_Point(i, point.x, point.y, point.z);
    pc_data_.points.push_back(point);
  }
  RCLCPP_INFO(
    node_->get_logger(), "Loaded mesh '%s' (%d points, %d triangles)",
    filename.c_str(), surface_shape_.get_Npoint(), surface_shape_.get_Nvertex());
  return true;
}

void IfsAlgorithm::apply_FaceGains(
  const std::vector<float> & raw, const std::vector<float> & point,
  std::vector<float> & out) const
{
  out = raw;
  if (!contact_correction_) {
    return;
  }
  const bool have_point =
    !(point[0] == 0.0F && point[1] == 0.0F && point[2] == 0.0F);
  if (!have_point) {
    return;
  }

  if (point[2] < kSideFaceMinZ) {
    // 윗면 — Mx(세로), My(가로)를 고친다. 둘 다 보정 전 값으로 계산한다.
    if (!kTopWrenchCorrection) {
      return;
    }
    const Wrench6 top = get_TopFormula(raw, point, false, gain_x_sign_, get_GainSet(sensor2_gains_));
    out[3] = static_cast<float>(top[3]);
    out[4] = static_cast<float>(top[4]);
    return;
  }

  // 옆면 — Fy, Mx, My, Mz 를 고친다 (My 는 side_my_from_* 가 0 이 아닐 때만 바뀜).
  if (side_gains_off_) {
    return;
  }
  const Wrench6 side = get_SideFormula(raw, point, false, gain_x_sign_, get_GainSet(sensor2_gains_));
  out[1] = static_cast<float>(side[1]);
  out[3] = static_cast<float>(side[3]);
  out[4] = static_cast<float>(side[4]);
  out[5] = static_cast<float>(side[5]);
}

// 모서리 넘어가기 대책(kEdgeTransitionActive)의 배율 반복. ft_value_ 로 ft_corrected_ 를 만든다.
// probe 는 들어올 때 직전 접촉점(cp_), 나갈 때 마지막 배율을 정한 위치다.
void IfsAlgorithm::apply_EdgeTransitionGains(std::vector<float> & probe)
{
  face_blend_used_ = false;
  // 배율 없이 구한 접촉점. 위치는 치우쳐 있지만 윗면에서 옆면까지 끊기지 않고 이어져서
  // 면을 정하는 데만 쓴다. 출력에 따라 바뀌지 않으므로 판정이 왕복할 수 없다.
  const std::vector<float> ref = get_NearestSurfacePoint(ft_value_, kNearestRefMaxDist);
  if (is_ZeroPoint(ref)) {
    // 마찰원뿔 안에 드는 면이 하나도 없다 (당기는 힘 등). 접촉점도 안 나온다.
    // 배율을 안 걸었으므로 probe 도 비워 둔다 (로그 라벨 none).
    ft_corrected_ = ft_value_;
    probe.assign(3, 0.0F);
    return;
  }

  if (ref[1] < 0.0F) {
    // 반대쪽 모서리 — 예전 면 식 그대로, 탐색만 가까운 표면점.
    for (int i = 0; i < kSideGainIterations; ++i) {
      apply_FaceGains(ft_value_, probe, ft_corrected_);
      const std::vector<float> next = get_NearestSurfacePoint(ft_corrected_, kNearestMaxDist);
      if (is_ZeroPoint(next)) {
        break;
      }
      probe = next;
    }
    apply_FaceGains(ft_value_, probe, ft_corrected_);
    return;
  }

  face_weights_ = get_FaceWeights(ft_value_, ref);
  face_blend_used_ = true;
  if (is_ZeroPoint(probe)) {
    // 새 접촉. 배율 안의 x, z 를 배율 없는 접촉점에서 시작한다.
    probe = ref;
  }
  for (int i = 0; i < kSideGainIterations; ++i) {
    apply_BlendedGains(
      ft_value_, probe, face_weights_, gain_x_sign_, get_GainSet(sensor2_gains_), ft_corrected_);
    const std::vector<float> next = get_NearestSurfacePoint(ft_corrected_, kNearestMaxDist);
    if (is_ZeroPoint(next)) {
      break;
    }
    probe = next;
  }
  apply_BlendedGains(
      ft_value_, probe, face_weights_, gain_x_sign_, get_GainSet(sensor2_gains_), ft_corrected_);
}

// 작용선에 가장 가까운 표면점 (모서리 넘어가기 대책 3).
// 마찰원뿔 안의 면만 보고, 비용 = 거리 + kNearestFacePenalty * (1 + n.f) 가 가장 작은
// 삼각형을 고른다. 그 거리가 max_distance 보다 크면 접촉점 없음 (0, 0, 0).
std::vector<float> IfsAlgorithm::get_NearestSurfacePoint(
  const std::vector<float> & ft, double max_distance, double rim_max_distance)
{
  std::vector<float> contact_point(3, 0.0F);
  const double fx = ft[0];
  const double fy = ft[1];
  const double fz = ft[2];
  const double squared_norm = fx * fx + fy * fy + fz * fz;
  if (squared_norm <= static_cast<double>(std::numeric_limits<float>::epsilon())) {
    return contact_point;
  }
  const double norm = std::sqrt(squared_norm);
  const Vec3 f {fx / norm, fy / norm, fz / norm};
  // 작용선이 지나는 점 r0 = (f x tau) / |f|^2  (get_WrenchAxis 와 같은 식)
  const Vec3 r0 {
    (fy * ft[5] - fz * ft[4]) / squared_norm,
    (fz * ft[3] - fx * ft[5]) / squared_norm,
    (fx * ft[4] - fy * ft[3]) / squared_norm};

  double best_cost = std::numeric_limits<double>::infinity();
  double best_distance = std::numeric_limits<double>::infinity();
  Vec3 best_point {0.0, 0.0, 0.0};
  for (int i = 0; i < surface_shape_.get_Nvertex(); ++i) {
    float nx = 0.0F;
    float ny = 0.0F;
    float nz = 0.0F;
    surface_shape_.get_Normal(i, nx, ny, nz);
    const Vec3 normal {nx, ny, nz};
    const double n_dot_f = dot3(normal, f);
    if (n_dot_f > -static_cast<double>(contact_cone_min_cos_)) {
      continue;  // 마찰원뿔 밖 (나가는 면 포함)
    }
    std::array<float, 3> p1 {};
    std::array<float, 3> p2 {};
    std::array<float, 3> p3 {};
    std::array<float, 3> unused_normal {};
    surface_shape_.get_Triangle(i, p1.data(), p2.data(), p3.data(), unused_normal.data());
    Vec3 closest {0.0, 0.0, 0.0};
    const double distance = get_LineTriangleDistance(
      r0, f, Vec3 {p1[0], p1[1], p1[2]}, Vec3 {p2[0], p2[1], p2[2]},
      Vec3 {p3[0], p3[1], p3[2]}, normal, closest);
    const double cost = distance + kNearestFacePenalty * (1.0 + n_dot_f);
    if (cost < best_cost) {
      best_cost = cost;
      best_distance = distance;
      best_point = closest;
    }
  }
  if (!(best_cost < std::numeric_limits<double>::infinity())) {
    return contact_point;
  }
  if (best_distance > max_distance) {
    // 옆면 테두리 점이면 한도를 rim_max_distance 까지 넓힌다 (kSensor2RimMaxDist, 센서2)
    const bool on_rim =
      best_point[1] > kSensor2RimMinY &&
      (std::abs(best_point[0]) >= kSensor2RimX || best_point[2] >= kSensor2RimZ);
    if (!(on_rim && best_distance <= rim_max_distance)) {
      return contact_point;
    }
  }
  contact_point[0] = static_cast<float>(best_point[0]);
  contact_point[1] = static_cast<float>(best_point[1]);
  contact_point[2] = static_cast<float>(best_point[2]);
  return contact_point;
}

void IfsAlgorithm::get_WrenchAxis(const std::vector<float> & ft, WrenchAxis & out)
{
  const float force_norm = std::sqrt(ft[0] * ft[0] + ft[1] * ft[1] + ft[2] * ft[2]);
  if (force_norm <= std::numeric_limits<float>::epsilon()) {
    insert_wrench(out.f, 0.0F, 0.0F, 0.0F);
    insert_wrench(out.r0, 0.0F, 0.0F, 0.0F);
    return;
  }

  insert_wrench(out.f, ft[0] / force_norm, ft[1] / force_norm, ft[2] / force_norm);
  const float squared_norm = force_norm * force_norm;
  insert_wrench(
    out.r0,
    (ft[1] * ft[5] - ft[2] * ft[4]) / squared_norm,
    (ft[2] * ft[3] - ft[0] * ft[5]) / squared_norm,
    (ft[0] * ft[4] - ft[1] * ft[3]) / squared_norm);
}

bool IfsAlgorithm::is_SurfaceContact(
  int index, WrenchAxis wrench_axis, std::vector<int> & vertex_indices,
  std::vector<float> & contact_point)
{
  vertex_indices.clear();

  int f1 = 0;
  int f2 = 0;
  int f3 = 0;
  std::array<float, 3> v1 {};
  std::array<float, 3> v2 {};
  std::array<float, 3> v3 {};
  std::array<float, 3> normal {};
  surface_shape_.get_TriangleIndex(index, f1, f2, f3);
  surface_shape_.get_Triangle(index, v1.data(), v2.data(), v3.data(), normal.data());

  const float denominator =
    wrench_axis.f.at[0] * normal[0] +
    wrench_axis.f.at[1] * normal[1] +
    wrench_axis.f.at[2] * normal[2];
  if (std::abs(denominator) <= std::numeric_limits<float>::epsilon()) {
    return false;
  }

  const float lambda =
    ((v1[0] * normal[0] + v1[1] * normal[1] + v1[2] * normal[2]) -
    (wrench_axis.r0.at[0] * normal[0] + wrench_axis.r0.at[1] * normal[1] +
    wrench_axis.r0.at[2] * normal[2])) / denominator;

  std::array<float, 3> center {};
  std::array<float, 3> p1 {};
  std::array<float, 3> p2 {};
  std::array<float, 3> p3 {};
  std::array<float, 3> l1 {};
  std::array<float, 3> l2 {};
  std::array<float, 3> l3 {};
  for (std::size_t i = 0; i < 3; ++i) {
    contact_point[i] = wrench_axis.r0.at[i] + lambda * wrench_axis.f.at[i];
    center[i] = (v1[i] + v2[i] + v3[i]) / 3.0F;
    p1[i] = 0.5F * (v2[i] + v3[i]);
    p2[i] = 0.5F * (v1[i] + v3[i]);
    p3[i] = 0.5F * (v2[i] + v1[i]);
    l1[i] = v2[i] - v3[i];
    l2[i] = v1[i] - v3[i];
    l3[i] = v1[i] - v2[i];
  }

  const auto norm = [](const std::array<float, 3> & vector) {
      return std::sqrt(
        vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
    };
  const float l1_norm = norm(l1);
  const float l2_norm = norm(l2);
  const float l3_norm = norm(l3);
  if (l1_norm <= 0.0F || l2_norm <= 0.0F || l3_norm <= 0.0F) {
    return false;
  }

  for (std::size_t i = 0; i < 3; ++i) {
    l1[i] /= l1_norm;
    l2[i] /= l2_norm;
    l3[i] /= l3_norm;
  }

  std::array<float, 3> n1 {};
  std::array<float, 3> n2 {};
  std::array<float, 3> n3 {};
  const auto projection = [](const std::array<float, 3> & a, const std::array<float, 3> & b) {
      return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    };
  std::array<float, 3> center_to_p1 {};
  std::array<float, 3> center_to_p2 {};
  std::array<float, 3> center_to_p3 {};
  for (std::size_t i = 0; i < 3; ++i) {
    center_to_p1[i] = center[i] - p1[i];
    center_to_p2[i] = center[i] - p2[i];
    center_to_p3[i] = center[i] - p3[i];
  }
  const float projection1 = projection(l1, center_to_p1);
  const float projection2 = projection(l2, center_to_p2);
  const float projection3 = projection(l3, center_to_p3);
  for (std::size_t i = 0; i < 3; ++i) {
    n1[i] = center_to_p1[i] - projection1 * l1[i];
    n2[i] = center_to_p2[i] - projection2 * l2[i];
    n3[i] = center_to_p3[i] - projection3 * l3[i];
  }

  const auto is_inside_edge = [&contact_point](
    const std::array<float, 3> & direction, const std::array<float, 3> & midpoint) {
      return direction[0] * (contact_point[0] - midpoint[0]) +
             direction[1] * (contact_point[1] - midpoint[1]) +
             direction[2] * (contact_point[2] - midpoint[2]) > 0.0F;
    };

  int inside_count = 0;
  if (is_inside_edge(n1, p1)) {
    vertex_indices.push_back(f1);
    ++inside_count;
  }
  if (is_inside_edge(n2, p2)) {
    vertex_indices.push_back(f2);
    ++inside_count;
  }
  if (is_inside_edge(n3, p3)) {
    vertex_indices.push_back(f3);
    ++inside_count;
  }
  // 삼각형 안에 들어와도 마찰원뿔 밖의 면이면 접촉으로 보지 않는다.
  // (이웃 삼각형으로 넘어가는 탐색은 vertex_indices 로 그대로 이어진다.)
  return inside_count == 3 && denominator <= -contact_cone_min_cos_;
}

int IfsAlgorithm::get_NextTriangle(int index, std::vector<int> & vertex_indices)
{
  if (vertex_indices.empty()) {
    return index;
  }

  std::array<int, 3> current {};
  surface_shape_.get_TriangleIndex(index, current[0], current[1], current[2]);

  for (int i = 0; i < surface_shape_.get_Nvertex(); ++i) {
    if (i == index) {
      continue;
    }
    std::array<int, 3> candidate {};
    surface_shape_.get_TriangleIndex(i, candidate[0], candidate[1], candidate[2]);
    const auto contains = [&candidate](int vertex) {
        return candidate[0] == vertex || candidate[1] == vertex || candidate[2] == vertex;
      };
    if (!contains(vertex_indices[0])) {
      continue;
    }

    if (vertex_indices.size() == 2 && contains(vertex_indices[1])) {
      return i;
    }
    if (vertex_indices.size() == 1) {
      int shared_vertices = 0;
      for (int candidate_vertex : candidate) {
        for (int current_vertex : current) {
          if (candidate_vertex == current_vertex) {
            ++shared_vertices;
          }
        }
      }
      if (shared_vertices == 1) {
        return i;
      }
    }
  }
  return index;
}

std::vector<float> IfsAlgorithm::get_ContactPoint(const std::vector<float> & ft)
{
  std::vector<float> contact_point(3, 0.0F);
  float absolute_sum = 0.0F;
  for (float value : ft) {
    absolute_sum += std::abs(value);
  }
  if (absolute_sum <= std::numeric_limits<float>::epsilon()) {
    return contact_point;
  }

  WrenchAxis wrench_axis {};
  get_WrenchAxis(ft, wrench_axis);
  int index = get_ClosestSurface(wrench_axis);
  std::vector<int> vertex_indices;
  for (int iteration = 0; iteration <= 100; ++iteration) {
    if (is_SurfaceContact(index, wrench_axis, vertex_indices, contact_point)) {
      return contact_point;
    }
    const int next_index = get_NextTriangle(index, vertex_indices);
    if (next_index == index) {
      break;
    }
    index = next_index;
  }
  return std::vector<float>(3, 0.0F);
}

void IfsAlgorithm::draw_arrow(
  const geometry_msgs::msg::PointStamped & contact_point,
  const geometry_msgs::msg::Wrench & force)
{
  visualization_msgs::msg::Marker arrow;
  arrow.header.stamp = node_->now();
  arrow.header.frame_id = contact_frame_;
  arrow.ns = "contact_force";
  arrow.id = 0;
  arrow.type = visualization_msgs::msg::Marker::ARROW;

  const double magnitude = std::sqrt(
    force.force.x * force.force.x +
    force.force.y * force.force.y +
    force.force.z * force.force.z);
  if (magnitude <= std::numeric_limits<double>::epsilon()) {
    arrow.action = visualization_msgs::msg::Marker::DELETE;
    arrow_pub_->publish(arrow);
    return;
  }

  first_draw_ = false;
  arrow.action = visualization_msgs::msg::Marker::MODIFY;
  arrow.color.r = 1.0F;
  arrow.color.g = 0.0F;
  arrow.color.b = 0.0F;
  arrow.color.a = 1.0F;

  constexpr double length_ratio = 0.007;
  arrow.scale.x = magnitude * length_ratio;
  arrow.scale.y = magnitude * length_ratio * 0.1;
  arrow.scale.z = magnitude * length_ratio * 0.1;
  arrow.pose.position.x = contact_point.point.x - force.force.x * length_ratio;
  arrow.pose.position.y = contact_point.point.y - force.force.y * length_ratio;
  arrow.pose.position.z = contact_point.point.z - force.force.z * length_ratio;

  const Eigen::Vector3d x_axis(1.0, 0.0, 0.0);
  const Eigen::Vector3d force_vector(force.force.x, force.force.y, force.force.z);
  const Eigen::Quaterniond quaternion =
    Eigen::Quaterniond::FromTwoVectors(x_axis, force_vector.normalized());
  arrow.pose.orientation.w = quaternion.w();
  arrow.pose.orientation.x = quaternion.x();
  arrow.pose.orientation.y = quaternion.y();
  arrow.pose.orientation.z = quaternion.z();
  arrow_pub_->publish(arrow);
}

void IfsAlgorithm::delete_arrow(
  const geometry_msgs::msg::PointStamped & contact_point,
  const geometry_msgs::msg::Wrench & force)
{
  (void)contact_point;
  (void)force;
  visualization_msgs::msg::Marker arrow;
  arrow.header.stamp = node_->now();
  arrow.header.frame_id = contact_frame_;
  arrow.ns = "contact_force";
  arrow.id = 0;
  arrow.type = visualization_msgs::msg::Marker::ARROW;
  arrow.action = visualization_msgs::msg::Marker::DELETE;
  arrow_pub_->publish(arrow);
}

void IfsAlgorithm::run()
{
  constexpr double loop_hz = 100.0;
  constexpr double rate_alpha = 0.2;
  rclcpp::Rate loop_rate(loop_hz);
  while (rclcpp::ok()) {
    pc_data_.header.stamp = node_->now();
    pointcloud_pub_->publish(pc_data_);

    if (ft_measure_flag_) {
      geometry_msgs::msg::Wrench force;

      // 센서값 Fy, Mx, Mz에 위치별 배율을 건다. 배율이 접촉 위치로 정해지고
      // 그 위치가 다시 배율에 의존하므로 몇 번 되풀이해 수렴시킨다.
      std::vector<float> probe = cp_;
      if (edge_transition_active_) {
        apply_EdgeTransitionGains(probe);
      } else if (raw_position_gains_) {
        // 배율 위치 = 원본 렌치의 면 평면 교점. 반복 없음 (kSensor2RawPositionGains).
        if (!get_EntryFacePoint(ft_value_, probe)) {
          probe.assign(3, 0.0F);
        }
        apply_FaceGains(ft_value_, probe, ft_corrected_);
      } else {
        for (int i = 0; i < kSideGainIterations; ++i) {
          apply_FaceGains(ft_value_, probe, ft_corrected_);
          std::vector<float> next = get_ContactPoint(ft_corrected_);
          if (next[0] == 0.0F && next[1] == 0.0F && next[2] == 0.0F) {
            // 커버와 안 만났다. 켜져 있으면 면 평면 교점을 다음 배율 위치로만 쓴다
            // (접촉점으로 발행하지 않는다). 그것도 못 구하면 예전처럼 멈춘다.
            if (!kPlaneProbeOnMiss || !get_FacePlaneProbe(ft_corrected_, next)) {
              break;
            }
          }
          probe = next;
        }
        apply_FaceGains(ft_value_, probe, ft_corrected_);
      }

      const double corrected_magnitude = std::sqrt(
        static_cast<double>(ft_corrected_[0]) * ft_corrected_[0] +
        static_cast<double>(ft_corrected_[1]) * ft_corrected_[1] +
        static_cast<double>(ft_corrected_[2]) * ft_corrected_[2]);

      std_msgs::msg::Float64 magnitude_message;
      magnitude_message.data = corrected_magnitude;
      force_magnitude_pub_->publish(magnitude_message);

      // 접촉 임계값과 무관하게 매번 내보낸다. 접촉 판정은 받는 쪽이 한다.
      geometry_msgs::msg::WrenchStamped corrected_message;
      corrected_message.header.stamp = node_->now();
      corrected_message.header.frame_id = contact_frame_;
      corrected_message.wrench.force.x = ft_corrected_[0];
      corrected_message.wrench.force.y = ft_corrected_[1];
      corrected_message.wrench.force.z = ft_corrected_[2];
      corrected_message.wrench.torque.x = ft_corrected_[3];
      corrected_message.wrench.torque.y = ft_corrected_[4];
      corrected_message.wrench.torque.z = ft_corrected_[5];
      corrected_wrench_pub_->publish(corrected_message);

      if (force_magnitude_ > contact_force_threshold_) {
        if (!contact_session_active_) {
          contact_session_active_ = true;
          contact_point_attempts_ = 0;
          contact_point_zero_results_ = 0;
          consecutive_zero_results_ = 0;
          max_consecutive_zero_results_ = 0;
        }

        const double raw_rate = (force_magnitude_ - prev_force_) * loop_hz;
        prev_force_ = force_magnitude_;
        force_rate_ += (raw_rate - force_rate_) * rate_alpha;

        // 힘이 빠르게 줄고 있으면 위치를 얼린다. 들어갈 때와 나올 때 문턱을
        // 다르게 둬(히스테리시스) 경계에서 깜빡이지 않게 한다.
        // 힘이 다시 늘기 시작하면 만료 상태를 푼다 (새 접촉)
        if (freeze_expired_ && force_rate_ > 0.0) {
          freeze_expired_ = false;
          position_frozen_ = false;
          freeze_ticks_ = 0;
        }

        if (kFreezeEnterRate > 0.0 && !freeze_expired_) {
          if (!position_frozen_) {
            if (force_rate_ < -kFreezeEnterRate &&
              force_magnitude_ < kFreezeBelowForce)
            {
              position_frozen_ = true;
              freeze_ticks_ = 0;
            }
          } else {
            ++freeze_ticks_;
            if (force_rate_ > -kFreezeExitRate) {
              // 힘이 다시 안정됐다 -> 정상 추적으로 복귀
              position_frozen_ = false;
              freeze_ticks_ = 0;
            } else if (freeze_ticks_ > static_cast<int>(kFreezeMaxHoldSec * loop_hz)) {
              // 계속 줄기만 한다 -> 접촉이 끝난 것으로 처리한다
              freeze_expired_ = true;
            }
          }
        } else {
          position_frozen_ = false;
        }

        if (freeze_expired_) {
          // 계속 줄기만 해서 접촉이 끝난 것으로 본다. 임계값 아래와 같은 처리.
          cp_.assign(3, 0.0F);
          cp_pubdata_.point.x = 0.0;
          cp_pubdata_.point.y = 0.0;
          cp_pubdata_.point.z = 0.0;
          delete_arrow(cp_pubdata_, force);
          count_ = 0;
        } else if (position_frozen_) {
          // 위치는 마지막 값을 유지하고, 접촉과 힘은 그대로 보고한다.
          force.force.x = ft_corrected_[0];
          force.force.y = ft_corrected_[1];
          force.force.z = ft_corrected_[2];
          const bool have_point =
            !(cp_pubdata_.point.x == 0.0 && cp_pubdata_.point.y == 0.0 &&
              cp_pubdata_.point.z == 0.0);
          if (have_point) {
            draw_arrow(cp_pubdata_, force);
          } else {
            delete_arrow(cp_pubdata_, force);
          }
          ++count_;
        } else if (count_ > 3) {
          if (edge_transition_active_) {
            cp_ = get_NearestSurfacePoint(ft_corrected_, kNearestMaxDist);
          } else if (nearest_max_dist_ > 0.0) {
            cp_ = get_NearestSurfacePoint(ft_corrected_, nearest_max_dist_, rim_max_dist_);
          } else {
            cp_ = get_ContactPoint(ft_corrected_);
          }
          if (contact_correction_) {
            correct_ContactPoint(cp_, ft_corrected_);
          }
          constexpr double lambda = 1.0;
          const bool contact_was_zero =
            cp_pubdata_.point.x == 0.0 && cp_pubdata_.point.y == 0.0 && cp_pubdata_.point.z == 0.0;
          const bool result_is_zero = cp_[0] == 0.0F && cp_[1] == 0.0F && cp_[2] == 0.0F;

          ++contact_point_attempts_;
          if (result_is_zero) {
            ++contact_point_zero_results_;
            ++consecutive_zero_results_;
            max_consecutive_zero_results_ = std::max(
              max_consecutive_zero_results_, consecutive_zero_results_);
          } else {
            consecutive_zero_results_ = 0;
          }

          const double zero_rate = 100.0 * static_cast<double>(contact_point_zero_results_) /
            static_cast<double>(contact_point_attempts_);
          RCLCPP_INFO_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "CP solve: attempts=%llu success=%llu zero=%llu (%.1f%%), "
            "consecutive_zero=%llu max_consecutive_zero=%llu",
            static_cast<unsigned long long>(contact_point_attempts_),
            static_cast<unsigned long long>(
              contact_point_attempts_ - contact_point_zero_results_),
            static_cast<unsigned long long>(contact_point_zero_results_), zero_rate,
            static_cast<unsigned long long>(consecutive_zero_results_),
            static_cast<unsigned long long>(max_consecutive_zero_results_));

          if (contact_was_zero) {
            cp_pubdata_.point.x = cp_[0];
            cp_pubdata_.point.y = cp_[1];
            cp_pubdata_.point.z = cp_[2];
          } else if (!result_is_zero) {
            cp_pubdata_.point.x = lambda * cp_[0] + (1.0 - lambda) * cp_pubdata_.point.x;
            cp_pubdata_.point.y = lambda * cp_[1] + (1.0 - lambda) * cp_pubdata_.point.y;
            cp_pubdata_.point.z =
              lambda * cp_[2] + (1.0 - lambda) * cp_pubdata_.point.z;
          }

          force.force.x = ft_corrected_[0];
          force.force.y = ft_corrected_[1];
          force.force.z = ft_corrected_[2];
          draw_arrow(cp_pubdata_, force);
          if (result_is_zero) {
            delete_arrow(cp_pubdata_, force);
          }

          // 이번에 걸린 배율을 면에 맞게 보여준다.
          //   옆면: Fy, Mx, Mz 배율    윗면: Mx 배율, My 배율 (세 번째는 1)
          //   모서리 넘어가기 대책: 윗면, 옆면, 모서리 확률
          const bool have_probe =
            !(probe[0] == 0.0F && probe[1] == 0.0F && probe[2] == 0.0F);
          const bool on_top = have_probe && probe[2] < kSideFaceMinZ;
          const char * gain_label = "none";
          double applied_gain_a = 1.0;
          double applied_gain_b = 1.0;
          double applied_gain_c = 1.0;
          if (!contact_correction_) {
            gain_label = "off";
          } else if (face_blend_used_) {
            gain_label = "blend top/side/edge";
            applied_gain_a = face_weights_.top;
            applied_gain_b = face_weights_.side;
            applied_gain_c = face_weights_.edge;
          } else if (have_probe && !on_top && side_gains_off_) {
            gain_label = "side off";
          } else if (have_probe && !on_top) {
            const FaceGainSet & g = get_GainSet(sensor2_gains_);
            const double gain_x = std::min(
              std::max(gain_x_sign_ * static_cast<double>(probe[0]), g.side_min_x),
              g.side_max_x);
            const double gain_z = std::min(
              std::max(static_cast<double>(probe[2]), g.side_min_z), g.side_max_z);
            gain_label = "side Fy/Mx/Mz";
            applied_gain_a = g.side_fy_c + g.side_fy_s * gain_x;
            applied_gain_b = (g.side_mx_c + g.side_mx_s * gain_x) *
              (1.0 + g.side_mx_z_slope *
              std::max(0.0, std::min(
                std::max(static_cast<double>(probe[2]), g.side_min_z),
                g.side_mx_z_max) - g.side_mx_z0));
            applied_gain_c = g.side_mz_c + g.side_mz_s * gain_z;
          } else if (on_top && kTopWrenchCorrection) {
            const FaceGainSet & g = get_GainSet(sensor2_gains_);
            const double gain_x = std::min(
              std::max(gain_x_sign_ * static_cast<double>(probe[0]), g.top_min_x),
              g.top_max_x);
            gain_label = "top Mx/My";
            applied_gain_a = g.top_mx_c + g.top_mx_s * gain_x;
            applied_gain_b = g.top_my;
          } else if (on_top) {
            gain_label = "top position-fit";
          }

          RCLCPP_INFO_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 500,
            "Contact: [%.4f %.4f %.4f] m, Force: [%.3f %.3f %.3f] N, "
            "|F|=%.3f N (raw %.3f N, gain %s %.3f %.3f %.3f)",
            cp_pubdata_.point.x, cp_pubdata_.point.y, cp_pubdata_.point.z,
            force.force.x, force.force.y, force.force.z,
            corrected_magnitude, force_magnitude_,
            gain_label, applied_gain_a, applied_gain_b, applied_gain_c);
        }
        ++count_;
      } else {
        if (contact_session_active_ && contact_point_attempts_ > 0) {
          const double zero_rate =
            100.0 * static_cast<double>(contact_point_zero_results_) /
            static_cast<double>(contact_point_attempts_);
          RCLCPP_INFO(
            node_->get_logger(),
            "CP solve summary: attempts=%llu success=%llu zero=%llu (%.1f%%), "
            "max_consecutive_zero=%llu",
            static_cast<unsigned long long>(contact_point_attempts_),
            static_cast<unsigned long long>(
              contact_point_attempts_ - contact_point_zero_results_),
            static_cast<unsigned long long>(contact_point_zero_results_), zero_rate,
            static_cast<unsigned long long>(max_consecutive_zero_results_));
        }
        contact_session_active_ = false;
        cp_.assign(3, 0.0F);
        cp_pubdata_.point.x = 0.0;
        cp_pubdata_.point.y = 0.0;
        cp_pubdata_.point.z = 0.0;
        delete_arrow(cp_pubdata_, force);
        count_ = 0;
        position_frozen_ = false;
        freeze_expired_ = false;
        freeze_ticks_ = 0;
        prev_force_ = 0.0;
        force_rate_ = 0.0;
      }

      cp_pubdata_.header.stamp = node_->now();
      cp_pub_->publish(cp_pubdata_);

      // cp 가 유효한 좌표일 때만 true. 얼림 만료로 cp 가 0 이 된 경우
      // 힘은 임계값을 넘어도 false 가 되어 cp 와 모순되지 않는다.
      std_msgs::msg::Bool contact_state_message;
      contact_state_message.data =
        !(cp_pubdata_.point.x == 0.0 &&
        cp_pubdata_.point.y == 0.0 &&
        cp_pubdata_.point.z == 0.0);
      contact_state_pub_->publish(contact_state_message);
      ft_measure_flag_ = false;
    }

#ifdef LOG_DATA
    if (out_file_.is_open()) {
      for (float value : ft_value_) {
        out_file_ << value << '\t';
      }
      out_file_ << cp_[0] << '\t' << cp_[1] << '\t' << cp_[2] << '\n';
    }
#endif

    rclcpp::spin_some(node_);
    loop_rate.sleep();
  }
}
