# GitHub 仓库创建和上传指南

## 步骤1: 在GitHub上创建新仓库

1. 登录 GitHub (https://github.com)
2. 点击右上角的 "+" 按钮，选择 "New repository"
3. 填写仓库信息：
   - Repository name: `AssetX` 或 `assetx`
   - Description: `🧩 Multi-format robot simulation asset bridge tool - URDF ↔ MJCF ↔ USD ↔ Genesis JSON conversion and validation`
   - 选择 Public (公开) 或 Private (私有)
   - **不要** 勾选 "Add a README file"、"Add .gitignore"、"Choose a license" (因为我们已经有了)
4. 点击 "Create repository"

## 步骤2: 连接本地仓库到GitHub

创建仓库后，GitHub会显示类似这样的命令。在项目目录下执行：

```bash
# 添加远程仓库 (替换YOUR_USERNAME为你的GitHub用户名)
git remote add origin https://github.com/YOUR_USERNAME/AssetX.git

# 推送到GitHub
git branch -M main
git push -u origin main
```

## 步骤3: 验证上传

上传成功后，你应该能在GitHub仓库页面看到：

- ✅ 完整的项目文件结构
- ✅ README.md 显示项目介绍
- ✅ License 文件显示 MIT 许可证
- ✅ GitHub Actions 自动运行CI测试

## 可选：配置仓库设置

在仓库的 Settings 页面：

1. **Branches**: 设置 `main` 为默认分支，启用分支保护
2. **Actions**: 确保GitHub Actions已启用
3. **Pages**: 如果需要，可以启用GitHub Pages展示文档
4. **Topics**: 添加标签如 `robotics`, `simulation`, `urdf`, `mjcf`, `usd`, `asset-conversion`

## 下一步

- 📝 完善README中的安装和使用说明
- 🧪 等待CI测试结果
- 📦 考虑发布到PyPI: `pip install assetx`
- 🤝 邀请协作者或设置贡献指南

## 示例仓库URL结构
```
https://github.com/YOUR_USERNAME/AssetX
├── README.md          # 项目主页
├── docs/              # 详细文档
├── examples/          # 使用示例
├── tests/             # 测试文件
└── .github/           # GitHub配置
    ├── workflows/     # CI/CD
    └── ISSUE_TEMPLATE/ # Issue模板
```
