# 匿名审稿版上传指南

## 一、先选择提交方式

匿名审稿期间，优先使用会议允许的匿名补充材料下载入口，或经过实际下载验证的匿名镜像。普通 GitHub 主页会展示账号、提交历史等信息，单独更改提交用户名不能清除这些信息。

本目录是待上传内容；私人核查记录和原始包不属于待上传范围。发布前还需确认资产再分发权限。会议当前阶段是否允许追加材料，应以投稿系统及官方要求为准。

## 二、上传到已有 GitHub 仓库保存

以下操作适合保存项目代码；向审稿人提供什么入口，按第一节单独处理。先安装 Git、Git LFS，在终端切换到“匿名包”和“下载仓库”所在的共同父目录。将仓库地址占位符替换为自己的地址。

```powershell
git clone https://github.com/OWNER/REPOSITORY.git PhysTrace_upload
cd PhysTrace_upload
git lfs install --local
git config --local user.name "Anonymous Authors"
git config --local user.email "anonymous@example.invalid"
git config --local commit.gpgsign false
git config --local tag.gpgsign false
New-Item -ItemType Directory -Path assets -Force
Copy-Item -LiteralPath '..\PhysTrace_Anonymous' -Destination '.\assets\piper' -Recurse
git add assets/piper
git lfs ls-files
git status --short
```

前提：目标 `assets/piper` 尚未存在。如已存在，先比较现有内容，避免重复嵌套或混入旧版本。克隆文件夹名称若已占用，请使用新的名称。以上局部 Git 配置仅影响新提交，不改变账号身份和已有提交。

检查 `git lfs ls-files` 有模型条目。确认待提交只有预期的资料，再执行：

```powershell
git diff --cached --stat
git commit -m "Add anonymous PiPER simulation assets"
git log -1 --format=fuller
git push origin main
```

通过 GitHub 正常登录授权，不把密码、令牌写入命令或文件。大文件存储和下载额度以自己的 GitHub 套餐为准。

保留原仓库 README，在其中加入相对链接即可：

```markdown
## Simulation assets
[PiPER scene and usage](assets/piper/README.md)
```

本包在子目录中的 `.gitattributes` 和 `.gitignore` 仍有效；其 `.github/workflows` 位于子目录时不会自动运行。完整性检查可以在本地运行。若要启用 CI，应另外将工作流移到仓库根目录并调整工作目录。

## 三、提供匿名下载

可以考察 [Anonymous GitHub](https://anonymous.4open.science/) 等匿名镜像服务。使用前确认服务的授权范围、当前文件大小限制、Git LFS 支持情况以及链接有效期。本教程不保证服务会完整代理全部大模型文件。

建立匿名入口后，用退出登录的浏览器进行实测：

1. 页面、文件内容、下载跳转均不展示作者账号或机构信息。
2. 下载至少一个大型模型，核对文件大小；内容不是以 `version https://git-lfs.github.com/spec/v1` 开头的指针。
3. 下载完整目录，在其中运行 `python tools/check_repository.py --full`。
4. 在 Isaac Sim 环境执行 `bash scripts/launch.sh --validate --seconds 10`，确认能够加载。
5. 冻结审稿快照，记录提交版本；更新须遵守会议当前阶段的规则。

如果匿名镜像不能完整提供 LFS 资产，选择会议允许的匿名附件或匿名大文件托管。不要将审稿链接跳转到带作者身份的个人 Release 页面。

## 四、为何推荐完整下载校验

Git LFS 将大文件内容放在独立存储中。普通网页上传和某些 ZIP 下载可能只包含指针文件；只有从审稿人实际入口下载并通过完整性检查，才能确认资料完整可用。

参考：[ICLR 2027 作者指南](https://iclr.cc/Conferences/2027/AuthorGuidelines)、[GitHub Git LFS 说明](https://docs.github.com/en/repositories/working-with-files/managing-large-files/about-git-large-file-storage)。
