# 자율 주차 시스템 문서

## 프로젝트 개요

이 프로젝트는 차량 동역학, 상태 및 입력 제약, 장애물을 고려하여 주차
궤적과 제어 입력을 생성하고 시각화합니다. 플래너는 RRT*로 충돌이 없는
기하학적 초기 경로를 생성한 뒤, OSQP로 QP(Quadratic Program)를 반복해서
풀며 경로를 보정합니다. 이 과정은 SQP와 유사한 구조를 사용합니다.

## 개발 환경 및 기술

- **언어**: C++
- **라이브러리**: Eigen, OSQP, OsqpEigen
- **도구**: VS Code, Docker, CMake
- **환경**: Ubuntu 22.04

## 알고리즘

경로 계획은 다음 두 단계로 구성됩니다.

1. **RRT***는 $(x, y, \theta)$ 공간에서 기하학적 경로를 생성합니다. 충돌
   검사는 차량을 차량 대각선 길이의 절반을 반지름으로 하는 원으로
   근사합니다. 생성된 경로는 예측 구간의 샘플 수에 맞게 재샘플링됩니다.
2. **반복 QP 보정**은 생성된 경로를 초기값으로 사용하여 차량 모델과
   장애물 제약을 국소 선형화한 QP를 반복해서 풉니다. QP 업데이트의
   적용 크기는 merit function 기반 선 탐색으로 결정합니다.

현재 구현의 RRT*는 기하학적 플래너입니다. 차량의 전체 비선형 동역학,
속도 제약, 조향각 제약은 RRT* 단계에서 적용하지 않습니다.

## 시스템 모델

차량의 상태와 입력은 다음과 같습니다.

$$
\mathbf{x}(t) =
\begin{bmatrix}
x(t) & y(t) & \theta(t) & v(t) & \delta(t)
\end{bmatrix}^{T},
\qquad
\mathbf{u}(t) =
\begin{bmatrix}
a(t) & \dot{\delta}(t)
\end{bmatrix}^{T}.
$$

`SystemModel`에서 사용하는 5차원 운동학적 자전거 모델은 다음과 같습니다.

$$
\dot{\mathbf{x}} = f(\mathbf{x}, \mathbf{u}) =
\begin{bmatrix}
v\cos\theta \\
v\sin\theta \\
\dfrac{v\tan\delta}{L} \\
a \\
\dot{\delta}
\end{bmatrix},
$$

여기서 $L$은 설정된 차량 길이입니다. 모델은 2차 근사로 이산화됩니다.
이때 야코비안과 아핀 항은 각각 $\mathbf{A}_k$,
$\mathbf{B}_k$, $\mathbf{g}_k$로 반환됩니다.

$$
\mathbf{x}_{k+1} \approx
\mathbf{A}_k\mathbf{x}_k +
\mathbf{B}_k\mathbf{u}_k +
\mathbf{g}_k.
$$

## 최적화 문제

$N = \lceil T / \Delta t \rceil$라 하겠습니다. QP 결정 벡터는 모든 상태 및
입력 증분 뒤에 목표 슬랙과 장애물 슬랙을 포함합니다.

$$
\mathbf{z} =
\begin{bmatrix}
\Delta\mathbf{x}_0 \\ \vdots \\ \Delta\mathbf{x}_N \\
\Delta\mathbf{u}_0 \\ \vdots \\ \Delta\mathbf{u}_{N-1} \\
\mathbf{s}_{\mathrm{goal}} \\
\mathbf{s}_{\mathrm{obs}}
\end{bmatrix}.
$$

상태 변수는 $5(N+1)$개, 입력 변수는 $2N$개입니다. 여기에 5개의
목표 슬랙 변수와 각 상태 샘플의 각 장애물마다 하나씩 장애물 슬랙
변수가 추가됩니다. 현재 기준 궤적은 $\bar{\mathbf{x}}_k$와
$\bar{\mathbf{u}}_k$로 나타냅니다.

### 목적 함수

구현에서는 다음과 같은 이차 목적 함수를 사용합니다.

$$
\min_{\mathbf{z}}
\quad
\frac{1}{2}\mathbf{z}^{T}\mathbf{H}\mathbf{z}
 + \mathbf{q}^{T}\mathbf{z},
$$

설정된 대각 상태 가중치와 입력 가중치는 기준 상태 및 입력 값과 QP
증분을 함께 페널티에 반영합니다. 목표 슬랙과 장애물 슬랙에도 이차
페널티가 부여됩니다.

$$
\mathbf{H}_{\mathrm{goal}} =
\rho_{\mathrm{goal}}\mathbf{I}_5,
\qquad
\mathbf{H}_{\mathrm{obs}} =
\rho_{\mathrm{obs}}\mathbf{I}.
$$

따라서 실제 구현의 비용 함수는 선형 슬랙 페널티를 포함하는 연속시간
적분식이 아닙니다. `rho_goal`과 `rho_obs` 설정값이 이차 슬랙 페널티를
조절합니다.

### 등식 제약

초기 상태 증분은 업데이트 후 초기 상태가 설정된 초기 상태를 유지하도록
고정됩니다. 각 $k = 0, \ldots, N-1$에 대해 선형화된 동역학 제약은
다음과 같습니다.

$$
\Delta\mathbf{x}_{k+1}
- \mathbf{A}_k\Delta\mathbf{x}_k
- \mathbf{B}_k\Delta\mathbf{u}_k
= \mathbf{A}_k\bar{\mathbf{x}}_k
 + \mathbf{B}_k\bar{\mathbf{u}}_k
 + \mathbf{g}_k
 - \bar{\mathbf{x}}_{k+1}.
$$

최종 상태는 5차원 목표 슬랙을 통해 설정된 목표에 소프트 제약으로
연결됩니다.

$$
\Delta\mathbf{x}_N + \mathbf{s}_{\mathrm{goal}}
= \mathbf{x}_{\mathrm{goal}} - \bar{\mathbf{x}}_N.
$$

목표 슬랙에는 이차 페널티가 적용되지만, 현재 QP에서는 비음수 제약이
없습니다.

현재 구현에는 최종 입력을 고정하는 등식 제약이 없습니다. 입력은
$k = 0, \ldots, N-1$에서만 정의되며 입력 경계 제약을 받습니다.

### 상태 및 입력 경계

상태와 입력의 경계는 업데이트된 값에 적용됩니다.

$$
\mathbf{x}_{\min} - \bar{\mathbf{x}}_k
\leq \Delta\mathbf{x}_k
\leq \mathbf{x}_{\max} - \bar{\mathbf{x}}_k,
$$

$$
\mathbf{u}_{\min} - \bar{\mathbf{u}}_k
\leq \Delta\mathbf{u}_k
\leq \mathbf{u}_{\max} - \bar{\mathbf{u}}_k.
$$

상태 경계는 속도와 조향각에만 적용되며, 위치와 요각은 제약하지 않습니다.
입력 경계는 가속도와 조향각 변화율 제약을 포함합니다.

### 장애물 제약

각 장애물과 각 상태 샘플에 대해 현재 $(x, y)$ 위치에서 장애물 직사각형까지의
가장 가까운 점을 계산합니다. 이 거리를 $d_k$, 평면상의 단위 방향 벡터를
$\mathbf{n}_k$라고 하면 국소 제약은 다음과 같습니다.

$$
\mathbf{n}_k^T
\begin{bmatrix}
\Delta x_k \\ \Delta y_k
\end{bmatrix}
 + s_{\mathrm{obs},k}
\geq r_{\mathrm{vehicle}} + m - d_k,
$$

또한 다음 조건을 만족합니다.

$$
 s_{\mathrm{obs},k} \geq 0,
\qquad
 r_{\mathrm{vehicle}} = \frac{1}{2}\sqrt{L^2 + W^2}.
$$

여기서 $m$은 설정된 안전 여유 거리입니다. 장애물 제약은 점과
직사각형 사이 거리의 국소 선형화이며, 차량의 회전된 전체 외곽 형상에
대한 정확한 제약은 아닙니다. 각 장애물과 상태 샘플마다 하나의 장애물
슬랙 변수가 생성됩니다.

## SQP 유사 업데이트

각 외부 반복에서 옵티마이저는 현재 기준 궤적을 중심으로 QP를 구성하고
OSQP로 풉니다. merit function은 QP 목적 함수와 제약 위반량을 함께
반영합니다. 적용되는 업데이트는 다음과 같습니다.

$$
\mathbf{w}_{\mathrm{new}}
= \mathbf{w}_{\mathrm{current}}
+ \alpha\,\Delta\mathbf{w},
\qquad 0 < \alpha \leq 1,
$$

merit function이 개선되지 않으면 백트래킹을 통해 $\alpha$를 줄입니다.
결정 변수 증분의 최대 노름 또는 평균 노름이 구현된 수렴 임계값보다
작아지거나, 설정된 SQP 반복 횟수에 도달하면 반복을 종료합니다. 증분에
대한 수렴 임계값은 현재 구현에 하드코딩되어 있으며, 최대 반복 횟수는 YAML 설정에서
읽습니다.

## 결과

![demo3](assets/demo/demo3.gif)

생성된 궤적은 반환된 상태 및 입력 시퀀스를 사용하여 시뮬레이션할 수
있습니다. 차량 크기, 상태 및 입력 경계, 페널티, 안전 여유 거리, 장애물
형상 등의 시나리오와 플래너 매개변수는 YAML 파일로 설정합니다.
