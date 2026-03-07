# 跑打使用大全

## 1. 操作逻辑确认
- 默认模式：`SOTF_AUTO`（跑打自瞄模式）
- 副操按 `A`：切到 `SOTF_AUTO`
- 副操按 `B`：切到 `MANUAL`（纯手动瞄准）

## 2. 自瞄触发条件
- 只有在以下 AprilTag 任意一个被识别到时，才允许跑打自瞄接管底盘朝向：
  - `2,5,8,11,9,10,18,27,26,25,24,21`
- 若未识别到白名单 Tag，即使处于 `SOTF_AUTO`，也保持纯手动转向。

## 3. AdvantageScope 重点观测键
- `SOTF/Mode`
  - 当前模式字符串：`SOTF_AUTO` 或 `MANUAL`
- `SOTF/AutoModeEnabled`
  - 是否开启自动模式（A/B 状态）
- `SOTF/HasWhitelistedTag`
  - 是否看到白名单 Tag
- `SOTF/AutoAimActive`
  - 本周期是否真正启用自瞄（模式与Tag条件同时满足）
- `Vision/SOTF/HasWhitelistedTag`
  - 视觉子系统侧白名单判定
- `Vision/SOTF/WhitelistedTagIds`
  - 当前白名单配置，确认部署版本一致

## 4. 赛前快速联调流程
1. 上电后不按任何键，确认 `SOTF/Mode = SOTF_AUTO`。
2. 摄像头看不到白名单 Tag 时，推杆转向应为纯手动。
3. 摄像头对准白名单 Tag，确认 `SOTF/AutoAimActive = true`，底盘开始自动修正朝向。
4. 按 `B`，确认 `SOTF/Mode = MANUAL` 且 `SOTF/AutoAimActive = false`。
5. 再按 `A`，确认可恢复自动模式。

## 5. 常见问题排查
- 一直不进入自瞄：
  - 先看 `Vision/SOTF/HasWhitelistedTag` 是否为 `false`
  - 检查 PhotonVision 相机名与连线
  - 检查 Tag 是否在白名单里
- 按键无效：
  - 检查手柄映射是否为 Xbox（A/B）或 PS5（Cross/Circle）
  - 检查手柄端口是否正确
- 自瞄方向异常：
  - 检查里程计朝向与联盟镜像配置
  - 检查摄像头安装外参（`VisionConstants.robotToCamera0/1`）

## 6. PathPlanner 与跑打共存说明
- 本次改动仅影响遥控默认驾驶命令切换逻辑，不修改 PathPlanner 自动路径文件。
- Autonomous 下仍由 AutoBuilder/PathPlanner 命令主导，SOTF 状态机不干涉自动模式调度。
