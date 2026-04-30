# SLAM 前端模式与 EuRoC 回归

本文记录当前仓库支持的视觉前端模式、主要算法链路、运行入口，以及 Jetson 上 EuRoC Machine Hall 全量回归结果。

## 模式总览

| 手机端显示 | CLI / 配置值 | 后端链路 | 当前建议 |
| --- | --- | --- | --- |
| ORB | `orb` | ORB-SLAM3 原生 ORB、局部建图、回环、重定位 | 原生 ORB-SLAM3 对照模式 |
| ORB VPI Remap | `orb` + `slam.orb_accel=vpi-remap` / `--orb-accel vpi-remap` | 保留 ORB-SLAM3 原生 ORB 特征策略，仅将 stereo rectification remap 切到 VPI CUDA | 精度验证模式，当前不作为性能默认 |
| ORB CUDA | `orb` + `slam.orb_accel=cuda` / `--orb-accel cuda` | 使用 OpenCV CUDA ORB 提取，后端仍走 ORB-SLAM3 | 实验性能模式，需逐序列验证精度 |
| LK GFTT | `lk` + `slam.lk_xfeat_seeding=false` | GFTT/Shi-Tomasi 建点，网格均衡，LK 连续跟踪，Stereo 深度，PnP | 实时端优先 |
| LK GFTT Per-Frame | `lk_gftt_per_frame` + `--lk-per-frame-accel cpu` | 每帧重提 GFTT，帧间 LK，SGBM 深度，RANSAC PnP + iterative refine | CPU 精度基线 |
| LK GFTT VPI CUDA | `lk_gftt_per_frame` + `--lk-per-frame-accel vpi-cuda` | VPI CUDA remap / disparity / 可选 PyrLK，CPU PnP | Jetson GPU 重点模式 |
| LK + XFeat seed | `lk` + `slam.lk_xfeat_seeding=true` | XFeat 周期性提供双目 seed，后续仍走 LK / PnP | 实验模式 |

说明：
- Android 端只允许在非运行态切换特征前端；切换后会重启 SLAM 会话。
- `LK GFTT VPI CUDA` 和 `LK GFTT Per-Frame` 是同一个 Per-Frame 前端的不同加速后端。
- `ORB VPI Remap` 和 `ORB CUDA` 都属于 ORB 模式的加速/预处理后端，前者保留原生 ORB 特征，后者替换特征提取器。
- ORB 模式在 EuRoC 回归中使用 ORB-SLAM3 官方 stereo 外参配置 `config/euroc/stereo_orb_official.yaml`；LK 系列继续使用当前 LK 回归配置 `config/euroc/stereo_orb2500.yaml`，避免把两条几何假设混在一起。

## ORB 配置修正

历史五模式回归里 ORB 结果曾出现米级 ATE，根因不是 LK/VPI 修改破坏了 ORB 代码路径，而是 ORB 模式复用了 `stereo_orb2500.yaml`。该文件中的 `Stereo.T_c1_c2` 方向适配了 LK 侧几何链路，但对 ORB-SLAM3 原生 stereo 来说方向相反，导致 ORB 频繁 `Fail to track local map`、创建多张 map，官方轨迹导出只取最大 map 后表现为大漂移。

当前修正：
- 新增 `config/euroc/stereo_orb_official.yaml`，使用 ORB-SLAM3 官方 EuRoC stereo 外参方向。
- `scripts/run_jetson_euroc_mh_feature_compare.sh` 中只有 `orb` 模式使用 `EUROC_ORB_SETTINGS`，默认指向上述 ORB 官方配置。
- 离线 ORB 评估导出 ORB-SLAM3 官方 EuRoC 轨迹，再转换成统一 CSV 供仓库评估脚本使用。
- ORB 原始输出默认不再走 `StabilizeOutputPose`；如需旧稳定器，可显式设置 `SMART_DRONE_POSE_STABILIZER=1`。

## ORB GPU 加速状态

当前已给 ORB-SLAM3 增加分段耗时统计：`orb_track_ms`、`orb_extract_ms`、`orb_stereo_ms`。Jetson 抽样结果显示 ORB 提取占主要耗时，stereo matching 通常只有 1-2 ms，因此 GPU 化优先目标是 ORB extractor，而不是双目匹配。

当前支持两个 ORB 加速入口：

- `--orb-accel cuda` / `slam.orb_accel=cuda`：使用 OpenCV CUDA ORB 替代原生 ORB 提取器。该路径可降低部分提取耗时，但特征分布与 ORB-SLAM3 原生策略不同，MH_04 等困难序列需要单独看精度。
- `--orb-accel vpi-remap` / `slam.orb_accel=vpi-remap`：保留 ORB-SLAM3 原生 ORB extractor，只把 stereo rectification 的 `cv::remap` 换成 VPI CUDA remap。该路径用于验证“原生特征策略 + GPU 预处理”，不改变后端建图、回环、重定位逻辑。

编译入口：
```bash
./scripts/build.sh replay --jetson-orin-nx --opencv-cuda-orb --opencv-cuda-orb-root /path/to/opencv_cuda_orb
```

VPI Remap 需要构建时打开 VPI：
```bash
./scripts/build.sh replay --jetson-orin-nx --opencv-cuda-orb --opencv-cuda-orb-root /path/to/opencv_cuda_orb --reconfigure
```

当前实现的关键限制：VPI Remap 仍以 `cv::Mat` 作为 ORB-SLAM3 输入输出边界，因此每帧存在 host -> VPI/CUDA -> host 拷贝。MH_01 全量实测显示它能保持精度同级，但平均 track 耗时没有下降。真正要获得明显收益，需要把相机输入、rectified image、金字塔或特征提取继续留在 GPU/VPI 内部，减少回拷。

另有手动实验开关：
```bash
SMART_DRONE_ORB_CUDA_PYRAMID=1
```

它只把原生 ORB extractor 的金字塔 resize 移到 OpenCV CUDA，FAST、OctTree、orientation、descriptor 仍走 ORB-SLAM3 原生实现。MH_01 实测精度同级，但平均耗时更高，因此当前不随 `vpi-remap` 默认启用。

## EuRoC 回归环境

- 数据集：Jetson `~/euroc/machine_hall`
- 序列：`MH_01_easy` 到 `MH_05_difficult`
- 回放工具：`/home/nvidia/smart_drone_offline_replay`
- 评估脚本：`tests/euroc/evaluate_euroc_regression.py`
- ATE 对齐：SE3 刚体对齐，不做 Sim3 缩放
- RPE delta：10 帧
- 时间戳关联窗口：50 ms

运行入口：
```bash
EUROC_REPLAY_BIN=/home/nvidia/smart_drone_offline_replay \
EUROC_EVAL_ROOT=/home/nvidia/euroc_eval \
EUROC_MACHINE_HALL_ROOT=/home/nvidia/euroc/machine_hall \
EUROC_MODES="orb orb_vpi_remap orb_cuda lk_gftt_grid lk_gftt_per_frame lk_gftt_vpi_cuda lk_xfeat_seed" \
EUROC_OUT=/home/nvidia/euroc_eval/results/mh_feature_modes_current_all_5modes \
/home/nvidia/euroc_eval/scripts/run_jetson_euroc_mh_feature_compare.sh
```

## 当前结果

ORB 修正后全量结果目录：
`/home/nvidia/euroc_eval/results/orb_official_cfg_all_mh`

LK / VPI / XFeat 当前五模式结果目录：
`/home/nvidia/euroc_eval/results/mh_feature_modes_current_vpi_iter_r3_default`

ORB VPI Remap 冒烟对照目录：
`/home/nvidia/cuda_orb_test/results/orb_vpi_remap_script_check_20260428_104222`

同一新二进制的 CPU ORB 冒烟对照目录：
`/home/nvidia/cuda_orb_test/results/orb_cpu_script_check_20260428_104544`

| 模式 | 序列 | 匹配帧数 | ATE RMSE (m) | ATE Max (m) | RPE RMSE (m) |
| --- | --- | ---: | ---: | ---: | ---: |
| ORB | MH_01_easy | 3640 | 0.0392 | 0.1071 | 0.0113 |
| ORB | MH_02_easy | 3001 | 0.0484 | 0.1196 | 0.0105 |
| ORB | MH_03_medium | 2632 | 0.0465 | 0.1243 | 0.0208 |
| ORB | MH_04_difficult | 1978 | 0.0556 | 0.1766 | 0.0253 |
| ORB | MH_05_difficult | 2223 | 0.0512 | 0.1498 | 0.0212 |
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

### ORB 加速冒烟对照

| 模式 | 序列 | 帧数 | ATE RMSE (m) | RPE RMSE (m) | Track 均值 (ms) | Extract 均值 (ms) | Stereo 均值 (ms) |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| ORB CPU | MH_01_easy | 3682 | 0.0477 | 0.0108 | 39.17 | 20.11 | 1.42 |
| ORB VPI Remap | MH_01_easy | 3682 | 0.0501 | 0.0109 | 40.20 | 19.78 | 1.44 |
| ORB VPI Remap + CUDA Pyramid | MH_01_easy | 3682 | 0.0502 | 0.0108 | 46.18 | 25.26 | 1.53 |

说明：`ORB VPI Remap` 日志应出现 `[orb_vpi_remap] backend=vpi_cuda size=752x480`。当前结果说明 VPI remap 路径已生效，精度与 CPU ORB 同级，但受 `cv::Mat` 边界拷贝影响，性能未提升。

## 结果解读

- ORB 修正后是 EuRoC Machine Hall 上精度最好的原生 SLAM 对照路径，说明原先 ORB 差主要是配置问题。
- `ORB VPI Remap` 保留原生 ORB 特征策略，MH_01 精度没有明显劣化；但当前没有性能收益，不建议作为默认实时模式。
- `LK GFTT VPI CUDA` 在困难序列上明显优于 CPU Per-Frame，是当前 Jetson GPU 重点优化方向。
- `LK GFTT Per-Frame CPU` 不依赖 VPI，仍是稳定 CPU 精度基线。
- `LK GFTT` 更适合实时端连续输出，但全局漂移高于 Per-Frame / ORB。
- `LK + XFeat seed` 当前没有显示出优于 GFTT seed 的回归收益。

## 指标解释

- ATE RMSE：整条轨迹经 SE3 刚体对齐后的绝对位置误差均方根，反映全局漂移。
- ATE Max：整条轨迹的最大绝对位置误差。
- RPE RMSE：间隔 10 帧的相对位移误差均方根，反映局部 VO 稳定性。
