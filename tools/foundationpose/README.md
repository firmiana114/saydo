# FoundationPose 源码入口

## 已接入

官方源码完整快照：`third_party/FoundationPose/`，固定到 `a1b694b83e633c2cb6115b9063d940a687759392`。共 54 个上游文件，包含推理、神经重建、C++/CUDA 扩展、环境与构建说明，以及许可证。保留上游代码原样；SayDo 集成代码放在本目录，不直接修改第三方源码。

版本及每文件 SHA-256：`third_party/FoundationPose.upstream.json`。从项目根目录验证：

```bash
python3 tools/foundationpose/check_source.py
```

该命令只验证源码，不导入 GPU 库、不安装依赖、不读取相机或机器人。通过不代表运行环境、权重或重建已就绪。

## 上游入口与 SayDo 数据

| 入口 | 用途 |
| --- | --- |
| `third_party/FoundationPose/estimater.py` | `FoundationPose.register()` 初始化，`track_one()` 连续跟踪 |
| `third_party/FoundationPose/run_demo.py` | 官方已有网格模型的离线演示 |
| `third_party/FoundationPose/bundlesdf/run_nerf.py` | 已知多视角相机位姿的神经重建 `run_neural_object_field()` |
| `tools/object_scan/capture_view.py` | SayDo 单视角 RGB-D 和基座系相机位姿采集 |

当前胶带没有 CAD，使用自带的神经重建入口。已采集数据在 `work/object_scan/tape_20260915/`，但尚需逐视角 mask、目标深度检查、物体参考系选取与数据适配。`base_T_camera` 是基座系相机位姿，后续必须明确转换为选定重建坐标系中的 `cam_in_obs`；不能直接将现有采集目录传给官方 YCB/LineMOD 示例。

上游 `run_nerf.py` 使用 RGB、米制深度、同像素坐标的 mask、真实 K 和相机位姿。内孔背景不应被填进胶带 mask。网格重建通过后，才将模型接入 register/track_one。暂不引入独立 BundleSDF 仓库的未知相机运动估计流程。

## 运行环境尚未部署

本次只接入源码。后续为 MSI GPU 建立独立 FoundationPose 环境，保持相机采集 `.venv` 的现有依赖。

按固定版本的 `readme.md`、`environment.yml` 与 `requirements.txt` 准备 PyTorch、CUDA Toolkit、PyTorch3D、NVDiffRast、Warp 和 C++ 构建依赖。必须区分 nvidia-smi 报告的驱动 CUDA 支持版本与实际 nvcc 工具链版本。

本项目需要无 CAD 重建，**不能只运行 `build_all_conda.sh` 就认为扩展完整**：该脚本只构建 mycpp，上游明确说明还需单独构建 `bundlesdf/mycuda`，并安装与 PyTorch/CUDA 匹配的 Kaolin。具体版本组合和构建命令需在 MSI 实际验证。

推理权重按上游 Data prepare 下载到以下本地目录（Git 已忽略）：

```text
third_party/FoundationPose/weights/2023-10-28-18-33-37/{config.yml,model_best.pth}
third_party/FoundationPose/weights/2024-01-11-20-02-45/{config.yml,model_best.pth}
```

准备好独立环境后，可先使用该环境的 Python 执行上游 `third_party/FoundationPose/check_env.py`。注意：上游检查把缺权重列为 warning，可能仍退出 0；必须另行确认两组权重存在，并用真实数据完成推理验证。官方 demo 也需要单独下载 demo_data。

上游 Docker 启动脚本包含固定宿主目录和容器参数，使用前需按本项目路径审阅；本次没有运行它，也没有启用任何控制机械臂的功能。

## 维护

更新源码时先选定新提交，对比兼容性，重新生成来源清单；不直接跟随 main 自动升级。权重、数据、编译产物和运行日志保留本地，实验输出使用项目 `work/` 下的独立目录。上游原始源码、LICENSE 和版权声明一起保留，许可证用途范围见上游 LICENSE 第 3.3 节。
