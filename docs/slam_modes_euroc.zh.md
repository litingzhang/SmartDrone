# SLAM 前端模式与 EuRoC 回归结果

本文记录当前仓库支持的视觉前端模式、核心算法链路、运行入口，以及已验证的 EuRoC Machine Hall 回归精度。

## 模式总览

| 手机端显示 | CLI / 配置值 | 后端链路 | 主要用途 | 当前建议 |
| --- | --- | --- | --- | --- |
| ORB | `orb` | ORB-SLAM3 原生 ORB 特征、地图点、局部建图、重定位 | 保留原生 ORB-SLAM3 能力与词袋路径 | 作为兼容和对照模式 |
| LK GFTT | `lk` + `slam.lk_xfeat_seeding=false` | GFTT/Shi-Tomasi 建点，网格均衡，LK 前向跟踪，Stereo 深度，PnP 位姿，LK 闭环/位姿平滑 | 实时运行，低算力，手机端默认 LK GFTT 路径 | 实时端优先 |
| LK + XFeat seed | `lk` + `slam.lk_xfeat_seeding=true` | 周期性用 XFeat 产生双目 seed，后续仍由 LK 跟踪和 PnP 估计位姿 | 纹理弱、GFTT 补点不足时的实验路径 | 仅实验，不作为默认 |
| LK GFTT Per-Frame | `lk_gftt_per_frame` + `--lk-per-frame-accel cpu` | 每帧重新 GFTT，LK 计算帧间光流，SGBM 取上一帧深度，RANSAC PnP 后用内点 `SOLVEPNP_ITERATIVE` 精修 | 离线回归、精度对比、稳定基线 | 当前 EuRoC 推荐基线 |
| LK GFTT VPI CUDA | `lk_gftt_per_frame` + `--lk-per-frame-accel vpi-cuda` | VPI CUDA remap / disparity，可选 VPI PyrLK；CPU 只接收少量特征点和视差结果做 PnP | Jetson 上验证 GPU 链路和吞吐 | 仍为实验模式 |

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
- CPU SGBM 仍是当前精度基线。

适用场景：

- Jetson 上验证 GPU 加速链路。
- 对比 CPU 和 VPI 的吞吐、延迟、精度。

局限：

- VPI disparity 参数对精度影响大，不同光照/纹理需要重新调。
- 当前不把它作为精度默认模式。

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
EUROC_MODES="lk_gftt_per_frame" \
EUROC_OUT=/home/nvidia/euroc_eval/results/mh_feature_modes_current_perframe_final \
/home/nvidia/euroc_eval/scripts/run_jetson_euroc_mh_feature_compare.sh
```

### 当前推荐基线：LK GFTT Per-Frame CPU

结果目录：

`/home/nvidia/euroc_eval/results/mh_feature_modes_current_perframe_final`

| 模式 | 序列 | 匹配帧数 | ATE RMSE (m) | ATE Max (m) | RPE RMSE (m) |
| --- | --- | ---: | ---: | ---: | ---: |
| LK GFTT Per-Frame CPU | MH_01_easy | 3640 | 0.1285 | 0.3230 | 0.0193 |
| LK GFTT Per-Frame CPU | MH_02_easy | 3001 | 0.2136 | 0.5905 | 0.0232 |
| LK GFTT Per-Frame CPU | MH_03_medium | 2632 | 0.4327 | 1.1467 | 0.0404 |
| LK GFTT Per-Frame CPU | MH_04_difficult | 1978 | 1.2648 | 2.1361 | 0.0637 |
| LK GFTT Per-Frame CPU | MH_05_difficult | 2223 | 1.0545 | 1.7389 | 0.0543 |

### 历史完整对比

以下结果来自历史完整回归目录：

`/home/nvidia/euroc_eval/results/mh_feature_modes_20260426_022448`

这些结果使用同一批历史 pose CSV 重新用当前评估脚本计算，用于横向理解各方案趋势；它们不是最新 Per-Frame PnP 精修后的结果。

| 模式 | 序列 | 匹配帧数 | ATE RMSE (m) | ATE Max (m) | RPE RMSE (m) |
| --- | --- | ---: | ---: | ---: | ---: |
| LK GFTT | MH_01_easy | 3640 | 0.3930 | 0.7389 | 0.0322 |
| LK GFTT | MH_02_easy | 3001 | 0.6913 | 1.7536 | 0.0433 |
| LK GFTT | MH_03_medium | 2632 | 1.1777 | 2.8703 | 0.1033 |
| LK GFTT | MH_04_difficult | 1978 | 1.7961 | 2.8571 | 0.1137 |
| LK GFTT | MH_05_difficult | 2223 | 2.1371 | 3.3612 | 0.0899 |
| LK GFTT Per-Frame | MH_01_easy | 3640 | 0.6731 | 1.9976 | 0.0429 |
| LK GFTT Per-Frame | MH_02_easy | 3001 | 0.5349 | 1.0559 | 0.0461 |
| LK GFTT Per-Frame | MH_03_medium | 2632 | 0.8967 | 2.5715 | 0.0668 |
| LK GFTT Per-Frame | MH_04_difficult | 1978 | 1.3847 | 2.4195 | 0.0757 |
| LK GFTT Per-Frame | MH_05_difficult | 2223 | 1.2670 | 1.8819 | 0.0685 |
| LK + XFeat seed | MH_01_easy | 3640 | 1.4651 | 3.5526 | 0.0592 |
| LK + XFeat seed | MH_02_easy | 3001 | 0.7099 | 1.6814 | 0.0528 |
| LK + XFeat seed | MH_03_medium | 2632 | 1.3951 | 2.8897 | 0.1712 |
| LK + XFeat seed | MH_04_difficult | 1978 | 2.6934 | 4.4329 | 0.1484 |
| LK + XFeat seed | MH_05_difficult | 2223 | 3.4469 | 5.0243 | 0.1662 |

## 指标解释

- ATE RMSE：整条轨迹经 SE3 刚体对齐后的绝对位置误差均方根，反映全局漂移。
- ATE Max：整条轨迹最大绝对位置误差。
- RPE RMSE：间隔 10 帧的相对位移误差均方根，反映局部帧间 VO 稳定性。

当前结论：

- `LK GFTT Per-Frame CPU` 是 EuRoC 当前精度最好、最稳定的基线。
- `LK GFTT` 更适合实时端，延迟和状态连续性更可控，但全局漂移比 Per-Frame 大。
- `LK + XFeat seed` 当前没有显示出优于 GFTT seed 的回归收益。
- `LK GFTT VPI CUDA` 仍需要继续以 Jetson 实测日志和全量回归为准，暂不作为精度默认。
