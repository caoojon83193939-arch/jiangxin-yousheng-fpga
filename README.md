# 匠心有声——非遗文化展馆多媒体导览终端

完赛源码提交包 · 2026-10-27

本包仅包含当前完赛工程及其必要依赖、最终位流、正式装卡素材和验证记录。没有旧版本顶层、历史阶段工程、回退位流或旧版装卡素材。

## 打开工程

完整解压至本地较短路径，用安路 TD 6.2.168116 打开根目录 `media_player.al`。器件 EG4S20BG256，顶层 top。

正式工程的 63 个 RTL 文件统一整理至 rtl/，4 个 IP 配置及启用的配套文件位于 ip/，引脚与时序约束位于 constraints/。这些硬件源文件、IP、约束及 ROM 均与已交付版本逐字节一致。本次仅调整工程引用路径与工程名称，未修改硬件功能逻辑。

## 目录

| 位置 | 内容 |
|---|---|
| media_player.al | 唯一主工程 |
| rtl/ | 当前工程实际使用的 RTL，按 ctrl、storage、audio、video、osd 等功能分类 |
| ip/ | PLL、异步 FIFO 配置与启用的生成文件 |
| constraints/ | top.adc 引脚约束、top.sdc 时序约束 |
| 根目录四个 .hex | 字形与频谱 ROM，硬件读取时使用固定文件名 |
| bitstream/media_player.bit | 已验证的最终下载位流 |
| tf_card/ | 最终 TF 卡的 13 个文件，采用真人解说 |
| tools/ | 独立构建与只读校验工具 |
| verification/ | 已有构建/回归记录、时序面积报告、迁移路径与哈希映射 |

## 直接上板

使用 `bitstream/media_player.bit`；可先 PROGRAM_SRAM，再根据板卡下载工具流程 PROGRAM_FLASH。

SHA256：56c676499061cf69b8fa334210254d07b08b3138325722151a3856b91c4db04c

将 tf_card/ 内 13 个文件直接放到 FAT32 卡根目录（保留 8.3 文件名，分配单元至少 2 KB）。图片、音乐与字幕沿用已验证素材；四段 NARR 真人解说为 2026-10-26 测试通过的版本。

SW2 下拨为展示、上拨为维护。展示界面固定双线性，维护界面可用 SW4 对比；展示 KEY2/KEY3 翻页，维护短按 KEY2/KEY3 调音量；KEY2+KEY3 按住 2 秒待机，任意键唤醒。数码管需 SW6 上拨。串口为 115200、8N1。

## 校验与构建

仅需 Python 3 标准库即可执行只读校验：

```powershell
python -B -X utf8 tools/verify_submission.py
```

重新生成主工程：

```powershell
python -B -X utf8 gen_project.py
```

命令行重新构建（将路径改为实际 TD 安装目录）：

```powershell
python -B -X utf8 tools/build.py --td-root "D:/Anlogic/TD_6.2.1_Engineer_6.2.168.116"
```

只执行 TD 设计读取检查：

```powershell
python -B -X utf8 tools/build.py --td-root "D:/Anlogic/TD_6.2.1_Engineer_6.2.168.116" --end-step read_design
```

构建脚本会复制 ROM 到 build/cli，应用正式 seed 3 与高力度参数，并使用既有时钟/引脚约束。GUI 构建时也需确保四个 .hex 可从 TD 的工作目录读取；推荐使用上述命令行脚本统一工作目录。

随包位流来自原正式构建，本次未重新完成全流程布局布线。调整文件位置和工程名称后重新编译的产物必须重新检查时序，不能沿用既有位流的报告。原报告 setup +0.001 ns、hold +0.004 ns、TNS 为 0。

## 记录与来源

verification/original_build_manifest.json 及日志是原始构建记录，包含旧路径、内部版本代号及当时“待上板”的状态，用于追溯，不代表本包有多个版本。用户后续已确认最新版本实板测试通过并提供约 28 小时运行记录；真人素材更换后另行确认播放正常，不将其宣称为又做了 28 小时长稳。未进行实际功耗测量。

verification/source_map.json 将本包源文件映射到原正式构建清单，可验证硬件文件完全相同。完整开发历史与依赖历史版本的全量回归脚本保存在原项目中，本提交包保留已有测试结果与记录，不含历史开发工程。

课程/板卡配套及安路工具生成的 IP、加密 HDMI 核、部分 SDRAM 模块保留原版权和内容，不表述为参赛者从零编写。
