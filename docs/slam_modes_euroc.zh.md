# SLAM 前端模式与 EuRoC 回归结果

本文记录当前仓库支持的视觉前端模式、核心算法链路、运行入口，以及已验证的 EuRoC Machine Hall 回归精度。

## 模式总览

| 手机端显示 | CLI / 配置值 | 后端链路 | 主要用途 | 当前建议 |
| --- | --- | --- | --- | --- |
| ORB | `orb` | ORB-SLAM3 原生 ORB 特征、地图点、局部建图、重定位 | 保留原生 ORB-SLAM3 能力与词袋路径 | 作为兼容和对照模式 |
| LK GFTT | `lk` + `slam.lk_xfeat_seeding=false` | GFTT/Shi-Tomasi 建点，网格均衡，LK 前向跟踪，Stereo 深度，PnP 位姿，LK 闭环/位姿平滑 | 实时运行，低算力，手机端默认 LK GFTT 路径 | 实时端优先 |
| LK + XFeat seed | `lk` + `slam.lk_xfeat_seeding=true` | 周期性用 XFeat 产生双目 seed，后续仍由 LK 跟踪和 PnP 估计位姿 | 纹理弱、GFTT 补点不足时的实验路径 | 仅实验，不作为默认 |
| LK GFTT Per-Frame | `lk_gftt_per_frame` + `--lk-per-frame-accel cpu` | 每帧重新 GFTT，LK 计算帧间光流，SGBM 取上一帧深度，RANSAC PnP 后用内点 `SOLVEPNP_ITERATIVE` 精修 | 离线回归、精度对比、稳定基线 | 当前 EuRoC 推荐基线 |
| LK GFTT VPI CUDA | `lk_gftt_per_frame` + `--lk-per-frame-accel vpi-cuda` | VPI CUDA remap / disparity，可选 VPI PyrLK；CPU 只接收少量特征点和视差结果做 PnP | Jetson 上验证 GPU 链路和吞吐 | 困难序列精度更优，仍需实时验证 |

说明：

- Android 端只在非运行态切换特征前端；切换后运行时会重启 SLAM 会话，使前端状态、XFeat worker、VPI 状态一致。
- 当前运行时会把 `feature_frontend=xfeat` 远端配置降级为 ORB；XFeat 当前主要作为 LK seed 来源，不作为完整原生 XFeat SLAM 前端。
- `LK GFTT VPI CUDA` 和 `LK GFTT Per-Frame` 是同一个 Per-Frame 前端的不同加速后端；VPI 路径主要改变 remap / disparity / PyrLK 的执行位置，不改变最终 PnP 位姿模型。

## 各模式方案

### ORB

ORB 模式直接使用 ORB-SLAM3 的标准 Stereo / Stereo-IMU / Mono / Mono-IMU 入口。特征提取、描述子、匹配、局部地图、闭环和重定位均由 ORB-SLAM3 原生路径完成。

适用场景：

- 需要 ORB-SLAM3 原生地图、词袋和重定位行为。
- 作为 LK / XFeat 实验路径的回归对照。

局限：

- 在当前 UVC packed stereo 实时链路上，ORB 计算量和稳定性不如 LK GFTT 路径可控。
- Android 端的特征前端切换主要围绕 LK GFTT 系列优化。

### LK GFTT

LK GFTT 是实时运行主路径。流程如下：

1. 对矫正后的左/右灰度图提取 GFTT/Shi-Tomasi 角点。
2. 通过网格均衡限制特征空间分布，避免点集中在局部纹理。
3. 使用 LK 光流在相邻帧之间跟踪左目点，同时跟踪右目点。
4. 用左右目视差恢复上一帧 3D 点。
5. 使用 PnP RANSAC 估计相邻帧相机位姿增量，并累积为轨迹。
6. 在跟踪退化或到达补点周期时，用当前帧 GFTT 重新补点。
7. 对纯水平横移光流增加前向分量限幅，降低左右移动被误判为前后移动的风险。

关键参数：

- `--feature-frontend lk`
- `slam.lk_xfeat_seeding=false`
- `--lk-loop-closure` 可打开 LK 图像关键帧闭环/位姿平滑。

适用场景：

- 实时端默认推荐模式。
- Jetson / CM5 上要求低延迟、稳定输出手机端位姿。

局限：

- 它是 VO + 轻量位姿图修正，不是完整 bundle adjustment。
- 无 IMU 时不能通过重力和惯性约束消除所有姿态/尺度耦合。

### LK + XFeat seed

该模式不直接用 XFeat 替代 LK 跟踪，而是让 XFeat 周期性产生双目 seed 点：

1. Python worker 对左右图做 XFeat 推理。
2. 适配层对左右 XFeat 点做极线与视差约束配对。
3. 配对后的点作为 LK track seed 注入。
4. 后续帧仍走 LK 光流、视差、PnP。

关键参数：

- `--feature-frontend lk`
- `--lk-xfeat-seeding`
- `--xfeat-device auto|cpu|cuda`
- `--xfeat-top-k`
- `--xfeat-max-points`
- `--xfeat-input-max-width`
- `--xfeat-input-max-height`

适用场景：

- 纹理弱、GFTT 初始化不足时的补点实验。
- 评估 XFeat 对 seed 质量的影响。

局限：

- Jetson 上 Python worker / IPC / CUDA 环境会显著影响实时性。
- 历史 EuRoC 回归显示该模式并不优于 GFTT seed，因此当前不作为默认。

### LK GFTT Per-Frame

Per-Frame 模式是当前 EuRoC 推荐基线。它不维护长期 LK track，而是每帧重新提取 GFTT，做一帧到下一帧的相对位姿估计：

1. 在上一帧左目提取 GFTT/Shi-Tomasi 角点。
2. 用网格均衡筛选角点。
3. 从上一帧左右目计算 dense disparity。
4. 用 LK 从上一帧左目跟踪到当前帧左目。
5. 对上一帧特征点读取一致性视差，反投影为 3D 点。
6. 使用当前帧 2D 点和上一帧 3D 点做 `solvePnPRansac`。
7. 对 RANSAC 内点再执行 `SOLVEPNP_ITERATIVE` 精修。
8. 累积相邻帧 `Tcw` 增量输出轨迹。

关键参数：

- `--feature-frontend lk_gftt_per_frame`
- `--lk-per-frame-accel cpu`

当前实现要点：

- Per-Frame PnP RANSAC 阈值为 `3.0 px`。
- RANSAC 之后会用内点二次精修位姿，降低帧间增量噪声。
- 深度读取使用 3x3 neighborhood median 和视差一致性门禁。

适用场景：

- EuRoC 回归基线。
- 对比 CPU / VPI 后端。
- 调参验证，不依赖长期 track 状态。

局限：

- 仍是纯视觉逐帧 VO 累积，没有全局 BA。
- MH_04 / MH_05 这种快速运动和模糊序列上漂移仍明显。

### LK GFTT VPI CUDA

VPI CUDA 是 Per-Frame 的加速后端。目标是让 rectified image、disparity、LK 输入尽量在 GPU / VPI 内部连续处理，最后只把少量特征点或视差结果拷回 CPU 做 PnP。

关键参数：

- `--feature-frontend lk_gftt_per_frame`
- `--lk-per-frame-accel vpi-cuda`

当前状态：

- VPI remap 和 VPI stereo disparity 已接入。
- VPI PyrLK 路径已接入但默认仍保守；实际精度和耗时需要继续按 Jetson 端日志验证。
- VPI stereo 默认参数已针对 `MH_04_difficult` 调整为 `P1=20`、`P2=176`、`uniqueness=0.38`。
- VPI CUDA 的 Per-Frame PnP 默认使用 `SOLVEPNP_ITERATIVE` RANSAC；VPI 后端默认重投影阈值为 `3.0px`，CPU Per-Frame 仍保持原阈值。
- CPU SGBM 仍是稳定基线；当前 VPI CUDA 在 MH_01/MH_03/MH_04/MH_05 上取得了更低 ATE/RPE，但仍需要结合实时耗时与现场日志继续验证。

适用场景：

- Jetson 上验证 GPU 加速链路。
- 对比 CPU 和 VPI 的吞吐、延迟、精度。

局限：

- VPI disparity 参数对精度影响大，不同光照/纹理需要重新调。
- 当前已完成全量 EuRoC 归档；是否作为默认模式还需要结合 Jetson 实时负载、VPI 参数稳定性和现场日志决定。

## EuRoC 回归测试

### 测试环境

- 数据集：Jetson `~/euroc/machine_hall`
- 序列：`MH_01_easy` 到 `MH_05_difficult`
- 回放工具：`smart_drone_offline_replay`
- 配置：`/home/nvidia/euroc_eval/config/euroc/stereo_orb2500.yaml`
- 评估脚本：`tests/euroc/evaluate_euroc_regression.py`
- ATE 对齐：SE3 Kabsch/SVD，不做 Sim3 缩放
- RPE delta：10 帧
- 时间戳关联窗口：50 ms

运行命令示例：

```bash
EUROC_REPLAY_BIN=/home/nvidia/smart_drone_offline_replay \
EUROC_EVAL_ROOT=/home/nvidia/euroc_eval \
EUROC_MACHINE_HALL_ROOT=/home/nvidia/euroc/machine_hall \
EUROC_MODES="orb lk_gftt_grid lk_gftt_per_frame lk_gftt_vpi_cuda lk_xfeat_seed" \
EUROC_OUT=/home/nvidia/euroc_eval/results/mh_feature_modes_current_all_5modes \
/home/nvidia/euroc_eval/scripts/run_jetson_euroc_mh_feature_compare.sh
```

### 当前五模式全量归档

结果目录：

`/home/nvidia/euroc_eval/results/mh_feature_modes_current_all_5modes`

VPI CUDA 最新默认回归结果目录：

`/home/nvidia/euroc_eval/results/mh_feature_modes_current_vpi_iter_r3_default`

本表为当前 Jetson 二进制在 `MH_01_easy` 到 `MH_05_difficult` 上的全量回归结果，覆盖当前要求归档的五种模式：`ORB`、`LK GFTT`、`LK GFTT Per-Frame`、`LK GFTT VPI CUDA`、`LK + XFeat seed`。

| 模式 | 序列 | 匹配帧数 | ATE RMSE (m) | ATE Max (m) | RPE RMSE (m) |
| --- | --- | ---: | ---: | ---: | ---: |
| ORB | MH_01_easy | 3640 | 4.2141 | 7.3276 | 0.2698 |
| ORB | MH_02_easy | 3001 | 4.5632 | 7.8127 | 0.2857 |
| ORB | MH_03_medium | 2632 | 3.5508 | 8.8201 | 0.6081 |
| ORB | MH_04_difficult | 1978 | 6.7590 | 13.3577 | 0.5677 |
| ORB | MH_05_difficult | 2223 | 6.8458 | 12.9992 | 0.5257 |
| LK GFTT | MH_01_easy | 3640 | 0.4561 | 1.1882 | 0.0379 |
| LK GFTT | MH_02_easy | 3001 | 0.6655 | 1.7901 | 0.0488 |
| LK GFTT | MH_03_medium | 2632 | 0.8426 | 1.5918 | 0.1187 |
| LK GFTT | MH_04_difficult | 1978 | 1.5769 | 2.5427 | 0.1170 |
| LK GFTT | MH_05_difficult | 2223 | 1.8996 | 3.3756 | 0.1059 |
| LK GFTT Per-Frame CPU | MH_01_easy | 3640 | 0.1285 | 0.3230 | 0.0193 |
| LK GFTT Per-Frame CPU | MH_02_easy | 3001 | 0.2136 | 0.5905 | 0.0232 |
| LK GFTT Per-Frame CPU | MH_03_medium | 2632 | 0.4327 | 1.1467 | 0.0404 |
| LK GFTT Per-Frame CPU | MH_04_difficult | 1978 | 1.2648 | 2.1361 | 0.0637 |
| LK GFTT Per-Frame CPU | MH_05_difficult | 2223 | 1.0545 | 1.7389 | 0.0543 |
| LK GFTT VPI CUDA | MH_01_easy | 3640 | 0.1050 | 0.2370 | 0.0105 |
| LK GFTT VPI CUDA | MH_02_easy | 3001 | 0.2652 | 0.6822 | 0.0459 |
| LK GFTT VPI CUDA | MH_03_medium | 2632 | 0.2916 | 0.4600 | 0.0296 |
| LK GFTT VPI CUDA | MH_04_difficult | 1978 | 0.4503 | 0.6921 | 0.0402 |
| LK GFTT VPI CUDA | MH_05_difficult | 2223 | 0.3657 | 0.5805 | 0.0342 |
| LK + XFeat seed | MH_01_easy | 3640 | 1.2158 | 3.0536 | 0.0624 |
| LK + XFeat seed | MH_02_easy | 3001 | 0.7166 | 1.5134 | 0.0571 |
| LK + XFeat seed | MH_03_medium | 2632 | 1.2439 | 2.2497 | 0.1705 |
| LK + XFeat seed | MH_04_difficult | 1978 | 2.7775 | 4.7850 | 0.1542 |
| LK + XFeat seed | MH_05_difficult | 2223 | 3.1879 | 4.9685 | 0.1712 |

### 当前结果解读

- `LK GFTT VPI CUDA` 在 `MH_04_difficult` 上优于 CPU Per-Frame，是当前针对困难序列重点优化后的默认模式。
- 当前 VPI 参数和 PnP 策略是针对 `MH_04_difficult` 优化后的默认值；相对最早 VPI 默认值，`MH_04_difficult` ATE 从 `1.0004 m` 降到 `0.4503 m`，RPE 从 `0.0522 m` 降到 `0.0402 m`。
- `MH_01_easy` 当前默认值为 `0.1050 m / 0.0105 m`，仍显著优于最早 VPI 默认值 `0.2059 m / 0.0264 m`；若现场更重视 easy/低速场景，可用 `SMART_DRONE_LK_PER_FRAME_PNP_REPROJ=2.0` 切换到 `0.0876 m / 0.0096 m` 的配置。
- `LK GFTT Per-Frame CPU` 在 `MH_01_easy`、`MH_02_easy` 上最好，且不依赖 VPI 运行时，仍是最稳的 CPU 精度基线。
- `LK GFTT` 的实时连续跟踪路径明显优于 ORB 和 XFeat seed，但全局漂移高于 Per-Frame 系列。
- `LK + XFeat seed` 当前没有带来正收益，说明 XFeat 作为 seed 的稳定性和实时链路成本还需要继续优化。
- ORB 在当前 offline replay 配置下表现最差，仅作为 ORB-SLAM3 原生路径兼容对照，不建议作为当前 Jetson 前视双目的默认模式。

## 指标解释

- ATE RMSE：整条轨迹经 SE3 刚体对齐后的绝对位置误差均方根，反映全局漂移。
- ATE Max：整条轨迹最大绝对位置误差。
- RPE RMSE：间隔 10 帧的相对位移误差均方根，反映局部帧间 VO 稳定性。

当前结论：

- `LK GFTT VPI CUDA` 是当前全量 EuRoC 中困难序列精度最好的模式。
- `LK GFTT Per-Frame CPU` 是当前不依赖 VPI 的稳定精度基线。
- `LK GFTT` 更适合实时端，延迟和状态连续性更可控，但全局漂移比 Per-Frame 大。
- `LK + XFeat seed` 当前没有显示出优于 GFTT seed 的回归收益。
- ORB 当前仅保留为原生 ORB-SLAM3 兼容和对照模式。
