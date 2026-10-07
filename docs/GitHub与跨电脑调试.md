# GitHub 与另一台 Ubuntu 电脑

默认发布到当前账号的私有仓库 `ros2-a2-machine-tending`，以实际返回 URL 为准。Ubuntu 电脑需登录同一账号，或由仓库所有者添加协作者。

## 发布

`scripts/github_repo.py --check` 只查询账号和 CI；`--publish` 创建私有仓库、初始化本地 Git、提交并推送。凭据只在进程内使用，不输出或写入令牌。不覆盖已有同名远端、不强推、不修改全局 Git 身份。运行机器人不需要此脚本或 GitHub API 凭据。

```powershell
python scripts/github_repo.py --check
python scripts/github_repo.py --publish
```

## Ubuntu 克隆与启动

已配置 SSH 密钥时：

```bash
git clone git@github.com:zcl1105/ros2-a2-machine-tending.git ~/a2_ws
cd ~/a2_ws
bash scripts/build.sh
source install/setup.bash
ros2 launch a2_demo demo.launch.py
```

HTTPS 地址：`https://github.com/zcl1105/ros2-a2-machine-tending.git`。使用 GitHub CLI / Git Credential Manager 登录；GitHub 不支持账号密码作为 Git HTTPS 密码，也不要把令牌拼进 URL。

## 更新

```bash
cd ~/a2_ws
git status
git pull --ff-only
bash scripts/build.sh
source install/setup.bash
bash scripts/integration.sh
```

`git pull` 前先检查工作区，自己的修改先提交到个人调试分支。建议每人独立分支、PR 合并；更新后重新编译和验收，再录屏。

仓库提交源码、模型、接口、配置、文档和测试，忽略 build/install/log/results 与缓存。CI 上传结果为 Actions artifact。视频和大型 bag 单独分享，避免进入 Git 历史。

CI 使用 Ubuntu 22.04 runner 与 ROS Humble 容器，验证 colcon 编译、核心测试和五场景 ROS 通信。**没有图形显示，RViz 布局和实际视频需要 Ubuntu 桌面验证。** 以对应提交的 Actions 结果为准，不把推送成功当作测试通过。
