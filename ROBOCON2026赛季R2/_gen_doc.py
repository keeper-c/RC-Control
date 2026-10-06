import docx
from docx import Document
from docx.shared import Inches, Pt, Cm, RGBColor
from docx.enum.text import WD_ALIGN_PARAGRAPH
import os

doc = Document()

# ---- Normal style ----
style = doc.styles["Normal"]
font = style.font
font.name = "Arial"
font.size = Pt(10.5)

# ---- Cover ----
doc.add_paragraph()
doc.add_paragraph()
title = doc.add_paragraph()
title.alignment = WD_ALIGN_PARAGRAPH.CENTER
run = title.add_run("RC-R2 机器人控制代码架构文档")
run.bold = True
run.font.size = Pt(24)
run.font.color.rgb = RGBColor(0x1A, 0x47, 0x8A)

subtitle = doc.add_paragraph()
subtitle.alignment = WD_ALIGN_PARAGRAPH.CENTER
run = subtitle.add_run("—— 新人快速接手指南 ——")
run.font.size = Pt(14)
run.font.color.rgb = RGBColor(0x66, 0x66, 0x66)

doc.add_paragraph()
info = doc.add_paragraph()
info.alignment = WD_ALIGN_PARAGRAPH.CENTER
run = info.add_run("2026 Robocon RC-R2 机器人\n版本 V2.1 | 2026/05/08")
run.font.size = Pt(11)
run.font.color.rgb = RGBColor(0x88, 0x88, 0x88)

doc.add_page_break()

# ===================================================================
# 1. Overview
# ===================================================================
doc.add_heading("一、项目整体架构", level=1)

doc.add_paragraph(
    "本项目采用 两层六文件 的分层架构：Rob_core（核心层）定义类型系统和对象封装，"
    "app（应用层）实现具体业务逻辑。两层通过 rob_core.h 中的 Robot_t 全局实例 RC_R2 进行解耦通信。"
)

doc.add_paragraph()

arch_text = (
    "+-----------------------------------------------------------+\n"
    "|                  App 应用层（业务实现）                      |\n"
    "|  rob_brain.h/c  <-- 决策调度（顶层状态机，切换行为）          |\n"
    "|       | 控制                                               |\n"
    "|  rob_chs.h/c    <-- 底盘状态机实现（entry/exit/task）      |\n"
    "|  rob_act.h/c    <-- 上层动作实现（entry/exit/task）        |\n"
    "|       | 注册到                                              |\n"
    "|-----------------------------------------------------------|\n"
    "|               Rob_core 核心层（类型系统）                     |\n"
    "|  rob_core.h  <-- 核心封装层（派生类 + Robot_t + RC_R2）     |\n"
    "|       ^ 组合自                                              |\n"
    "|  rob_power.h <-- 动力接口层（函数指针表，抽象硬件驱动）       |\n"
    "|  rob_behave.h <-- 行为基类层（状态/动作结构体 + 枚举）       |\n"
    "+-----------------------------------------------------------+"
)
p = doc.add_paragraph()
run = p.add_run(arch_text)
run.font.name = "Consolas"
run.font.size = Pt(8.5)

doc.add_paragraph()
doc.add_paragraph(
    "核心设计思想：参照 C++ 基类->派生类->对象实例化 的思想，用 C 语言的结构体和函数指针模拟面向对象。\n"
    "  - rob_behave.h 定义"基类"（ChsState_t / Action_t）\n"
    "  - rob_power.h 定义硬件接口抽象（函数指针表，类似虚函数表）\n"
    "  - rob_core.h 定义"派生类"（RobotChs_t / RobotAct_t / Robot_t），并在 rob_core.c 中实例化为 RC_R2\n"
    "  - 外部模块只需 #include \"rob_core.h\" 即可拿到完整类型系统"
)

doc.add_page_break()

# ===================================================================
# 2. Rob_core Layer
# ===================================================================
doc.add_heading("二、Rob_core 核心层 — 类型系统与对象封装", level=1)

doc.add_heading("2.1 rob_behave.h — 行为基类层（最底层）", level=2)
doc.add_paragraph(
    "职责：定义状态机的基础类型，是整个类型系统的基石。不依赖任何 rob_* 文件。\n\n"
    "核心类型：\n"
    "  - ChsState_t：底盘状态基类（id + entry/exit/Task 三个函数指针）\n"
    "  - Action_t：动作基类（结构同上，用于上层机构）\n"
    "  - ChsState_e：底盘状态枚举（Idle 空闲 / StepUpDown 上下台阶 / Climb 爬升 / AutoTrack 视觉跟踪 / ApproachKFS 靠近基板 / Stop 停止）\n"
    "  - ActEvent_e：动作事件枚举（Idle 空闲 / GrabReady 准备抓取 / Grab 抓取 / Assembly 装配 / GantryGet 龙门架夹取）\n"
    "  - kRobEvt_*：FreeRTOS 任务通知事件位（Chassis=0x01 / Action=0x02 / Safety=0x04）"
)

doc.add_heading("2.2 rob_power.h — 动力接口层", level=2)
doc.add_paragraph(
    "职责：将底层硬件驱动抽象为统一的函数指针接口，供上层通过 RC_R2 调用。\n\n"
    "核心类型：\n"
    "  - ChsPower_t：底盘动力接口（Wheel 四轮麦轮速度控制 / Track 履带位置+速度控制）\n"
    "  - ActPower_t：上层机构动力接口（Gantry_lift 龙门架升降 / Gantry_push 龙门架推杆 / Gantry_grip 龙门架夹取）\n\n"
    "设计意图：更换硬件时只需修改 power_bsp.c 中的绑定函数，上层业务代码无需变动。"
)

doc.add_heading("2.3 rob_core.h — 核心封装层（三层体系的顶层）", level=2)
doc.add_paragraph(
    "职责：组合 behave 和 power 层，构建完整的派生类对象类型，对外暴露全局唯一机器人实例。\n\n"
    "核心类型：\n"
    "  - RobotChs_t：底盘派生类（name 名字 + now 当前状态 + last 上一状态 + pkg 状态表指针 + power 动力接口指针）\n"
    "  - RobotAct_t：上层机构派生类（结构同上）\n"
    "  - Robot_t：机器人顶层封装（name + eye 视觉标志 + Chs 底盘对象 + Act 上层机构对象）\n\n"
    "外部接口：\n"
    "  - extern Robot_t RC_R2;  —— 全局唯一机器人实例\n"
    "  - void RC_R2_Init(void);  —— 初始化函数\n\n"
    "使用约定：外部模块只需 #include \"rob_core.h\"，无需逐个引入子头文件。"
)

doc.add_heading("2.4 rob_core.c — 对象实例化与初始化", level=2)
doc.add_paragraph(
    "职责：定义静态配置表，完成 RC_R2 的实例化和初始化。\n\n"
    "关键内容：\n"
    "  1. s_chs_state_pkg[] — 底盘状态表，将枚举值映射到具体 entry/exit/task 函数\n"
    "  2. s_action_pkg[] — 动作状态表，同上\n"
    "  3. s_chs_power_cfg / s_act_power_cfg — 动力配置，绑定 BSP 层驱动函数（Wheel_config / Gantry_lift_config 等）\n"
    "  4. RC_R2_Init() — 将 now/last 初始化为 Idle 态，将静态底盘/动作对象赋值给 RC_R2\n\n"
    "新增状态或动作的标准步骤（五步法）：\n"
    "  1. rob_behave.h 中添加枚举值\n"
    "  2. rob_chs.h（或 rob_act.h）中声明对应的 entry/exit/task 函数\n"
    "  3. rob_chs.c（或 rob_act.c）中实现函数\n"
    "  4. rob_core.c 的状态表中注册\n"
    "  5. rob_brain.c 中添加对应的决策触发条件"
)

doc.add_page_break()

# ===================================================================
# 3. App Layer
# ===================================================================
doc.add_heading("三、App 应用层 — 业务逻辑实现", level=1)

doc.add_heading("3.1 rob_chs.h/c — 底盘状态机实现", level=2)
doc.add_paragraph(
    "职责：实现底盘 6 个状态的 entry（进入）/ exit（退出）/ Task（执行）函数，\n"
    "运行时由 Chs_MoveFunction 任务统一调度，周期 8ms。\n\n"
    "文件结构：\n"
    "  - rob_chs.h：对外声明所有 entry/exit/task 函数 + 工具函数 Chs_IsUpDownActive / Chs_UpDownSigReset\n"
    "  - rob_chs.c：具体实现 + 模块私有静态变量（s_speed_coeff 速度系数、s_updown_active 台阶状态、s_climb_timeout_* 超时监测等）\n\n"
    "执行流程（Chs_MoveFunction 任务循环）：\n"
    "  1. 检测 RC_R2.Chs.now.id != last.id -> 调用 last.exit() 清理 + now.entry() 初始化\n"
    "  2. 每周期调用 now.Task() 执行业务逻辑\n"
    "  3. 上下台阶超时安全监测（10s 超时自动关闭台阶机构）\n"
    "  4. 发送任务通知 xTaskNotify（kRobEvt_Chassis 位），触发决策层下一轮调度"
)

doc.add_heading("3.2 rob_act.h/c — 上层动作实现", level=2)
doc.add_paragraph(
    "职责：实现上层机构 5 个动作的 entry/exit/task 函数，\n"
    "运行时由 Act_ActionFunction 任务统一调度，周期 9ms。\n\n"
    "文件结构：\n"
    "  - rob_act.h：对外声明所有 entry/exit/task 函数 + GrabPkt_t 抓取参数结构体\n"
    "  - rob_act.c：具体实现 + 模块私有变量（s_action_timer 动作计时、s_grab_pkt 当前抓取参数、s_chassis_stop_pending 底盘停止标志、s_pid_posout/s_pid_spdout PID 中间结果等）\n\n"
    "执行流程（Act_ActionFunction 任务循环）：\n"
    "  1. 同底盘，检测状态切换 -> 调用 exit() + entry()\n"
    "  2. 执行 now.Task()\n"
    "  3. 执行小电机 PID 运动控制层（位置环 + 速度环级联）\n"
    "  4. 发送任务通知 xTaskNotify（kRobEvt_Action 位），触发决策层下一轮调度"
)

doc.add_heading("3.3 rob_brain.h/c — 决策调度（最高层）", level=2)
doc.add_paragraph(
    "职责：机器人顶层决策状态机，根据视觉数据（VisData）和坐标数据（Coord）\n"
    "决定底盘走哪个状态、上层机构执行哪个动作。\n\n"
    "核心函数 StateFlag_Function（FreeRTOS 任务）：\n"
    "  - 阻塞等待任务通知 ulTaskNotifyTake\n"
    "  - 根据通知位分三路处理：\n"
    "    1. kRobEvt_Chassis 位（0x01）—— 底盘决策：\n"
    "       按优先级判断：Stop > StepUpDown > Climb > AutoTrack > ApproachKFS > Idle\n"
    "       直接赋值 RC_R2.Chs.now = RC_R2.Chs.pkg[目标枚举]\n"
    "    2. kRobEvt_Action 位（0x02）—— 动作决策：\n"
    "       根据视觉跟踪信号触发 GrabReady / Grab / Assembly / GantryGet\n"
    "       直接赋值 RC_R2.Act.now = RC_R2.Act.pkg[目标枚举]\n"
    "    3. kRobEvt_Safety 位（0x04）—— 安全等待：\n"
    "       阻塞等待所有视觉跟踪信号归零后才继续\n\n"
    "关键决策依据：\n"
    "  - Coord.target.z 判断目标台阶高度（200/400/600 表示台阶，0 表示平地）\n"
    "  - VisData.Climb_Sig 爬升信号\n"
    "  - VisData.Wp_Trk_Sig / Asm_Trk_Sig / Kfs_Trk_Sig 视觉跟踪信号\n"
    "  - Coord.error 坐标误差、Coord.detect 检测坐标"
)

doc.add_page_break()

# ===================================================================
# 4. Key Data Structures
# ===================================================================
doc.add_heading("四、关键数据结构速查", level=1)

doc.add_heading("ChsState_t / Action_t（行为基类）", level=2)
code = (
    "typedef struct ChsState\n"
    "{\n"
    "    int8_t id;              // 状态 ID，对应 ChsState_e 枚举\n"
    "    void (*entry)(void);    // 进入状态时调用一次\n"
    "    void (*exit)(void);     // 离开状态时调用一次\n"
    "    void (*Task)(void);     // 持续期间每个周期调用\n"
    "} ChsState_t;\n"
    "\n"
    "// Action_t 结构完全相同，用于上层机构动作"
)
p = doc.add_paragraph()
run = p.add_run(code)
run.font.name = "Consolas"
run.font.size = Pt(9)

doc.add_paragraph()

doc.add_heading("Robot_t（顶层机器人封装）", level=2)
code = (
    "typedef struct Robot\n"
    "{\n"
    "    char *name;             // 机器人名称 \"2026RC-R2\"\n"
    "    int16_t eye;            // 视觉标志位\n"
    "    RobotChs_t Chs;          // 底盘（包含 now / last / pkg / power）\n"
    "    RobotAct_t Act;          // 上层机构（同上）\n"
    "} Robot_t;\n"
    "\n"
    "extern Robot_t RC_R2;       // 全局唯一机器人实例，所有模块通过它读写状态"
)
p = doc.add_paragraph()
run = p.add_run(code)
run.font.name = "Consolas"
run.font.size = Pt(9)

doc.add_paragraph()

doc.add_heading("状态表注册格式（以底盘为例）", level=2)
code = (
    "static ChsState_t s_chs_state_pkg[] =\n"
    "{\n"
    "    {kChsState_Idle,        Chs_IdleEntry,        Chs_IdleExit,        Chs_IdleTask},\n"
    "    {kChsState_StepUpDown,  Chs_StepUpDownEntry,  Chs_StepUpDownExit,  Chs_StepUpDownTask},\n"
    "    {kChsState_Climb,       Chs_ClimbEntry,       NULL,                Chs_ClimbTask},\n"
    "    {kChsState_AutoTrack,   NULL,                 NULL,                Chs_AutoTrackTask},\n"
    "    {kChsState_ApproachKFS, Chs_ApproachKFSEntry, NULL,                Chs_ApproachKFSTask},\n"
    "    {kChsState_Stop,        Chs_StopEntry,        NULL,                NULL},\n"
    "};\n"
    "// entry / exit / Task 均可为 NULL，表示该状态不需要对应回调"
)
p = doc.add_paragraph()
run = p.add_run(code)
run.font.name = "Consolas"
run.font.size = Pt(9)

doc.add_paragraph()

doc.add_heading("动力接口注册格式", level=2)
code = (
    "static ChsPower_t s_chs_power_cfg =\n"
    "{\n"
    "    .Wheel = Wheel_config,    // 四轮麦轮驱动函数\n"
    "    .Track = Track_config,    // 履带驱动函数\n"
    "};\n"
    "\n"
    "static ActPower_t s_act_power_cfg =\n"
    "{\n"
    "    .Gantry_lift = Gantry_lift_config,  // 龙门架升降\n"
    "    .Gantry_push = Gantry_push_config,  // 龙门架推杆\n"
    "    .Gantry_grip = Gantry_grip_config,  // 龙门架夹取\n"
    "};\n"
    "// 更换硬件时只需替换这些函数指针，上层代码无需修改"
)
p = doc.add_paragraph()
run = p.add_run(code)
run.font.name = "Consolas"
run.font.size = Pt(9)

doc.add_page_break()

# ===================================================================
# 5. Runtime Flow
# ===================================================================
doc.add_heading("五、运行时执行流程", level=1)

doc.add_paragraph(
    "系统启动后，FreeRTOS 调度器同时运行三个任务，形成闭环控制系统："
)
doc.add_paragraph()

flow = (
    "StateFlag_Function（决策层，rob_brain.c，事件驱动）\n"
    "  |  阻塞等待任务通知（ulTaskNotifyTake）\n"
    "  |  根据视觉/坐标数据决定目标状态\n"
    "  |  直接赋值：RC_R2.Chs.now = RC_R2.Chs.pkg[kChsState_XXX]\n"
    "  |            RC_R2.Act.now = RC_R2.Act.pkg[kActEvent_XXX]\n"
    "  |\n"
    "  +---> Chs_MoveFunction（底盘执行层，rob_chs.c，8ms 周期）\n"
    "  |       检测 now.id != last.id -> 调用 exit() + entry() 完成状态切换\n"
    "  |       每周期调用 now.Task() 执行底盘运动控制（麦轮/履带/视觉跟踪）\n"
    "  |       发送通知 kRobEvt_Chassis 位（0x01）\n"
    "  |\n"
    "  +---> Act_ActionFunction（动作执行层，rob_act.c，9ms 周期）\n"
    "          检测 now.id != last.id -> 调用 exit() + entry() 完成状态切换\n"
    "          每周期调用 now.Task() 执行时序动作\n"
    "          执行 PID 位置-速度级联电机控制\n"
    "          发送通知 kRobEvt_Action 位（0x02）"
)
p = doc.add_paragraph()
run = p.add_run(flow)
run.font.name = "Consolas"
run.font.size = Pt(8.5)

doc.add_paragraph()
doc.add_paragraph(
    "关键机制说明：\n"
    "  - 决策层只负责"选择目标"（now = pkg[XXX]），不直接调用动作函数\n"
    "  - 执行层检测到 now.id 变化后自动完成 exit() 清理 + entry() 初始化\n"
    "  - 同一状态的 Task() 函数每周期被调用直到下次状态切换\n"
    "  - 三层通过任务通知位精准唤醒，避免无谓的轮询"
)

doc.add_page_break()

# ===================================================================
# 6. Onboarding Guide
# ===================================================================
doc.add_heading("六、新人上手指南", level=1)

steps = [
    ("先理解三层类型体系（10 分钟）",
     "按 rob_behave.h -> rob_power.h -> rob_core.h 顺序阅读。\n"
     "理解 ChsState_t 基类 -> ChsPower_t 接口 -> RobotChs_t 派生类 -> Robot_t 顶层封装的层层组合关系。"),
    ("理解全局实例 RC_R2（5 分钟）",
     "RC_R2 是全局唯一的机器人对象，底盘和上层机构都通过它来读写状态。\n"
     "rob_core.c 中的 RC_R2_Init() 是唯一的初始化入口。"),
    ("跟踪一次完整的状态切换流程（15 分钟）",
     "以"空闲->爬升"为例：\n"
     "  StateFlag_Function 检测到 VisData.Climb_Sig == 1\n"
     "  -> 赋值 RC_R2.Chs.now = pkg[kChsState_Climb]\n"
     "  -> Chs_MoveFunction 检测到 id 变化\n"
     "  -> 调用 Chs_IdleExit()（停止麦轮）\n"
     "  -> 调用 Chs_ClimbEntry()（启动履带）\n"
     "  -> 每周期调用 Chs_ClimbTask()（履带+麦轮协同爬升）"),
    ("新增一个动作/状态的标准流程（参考实现）",
     "1. rob_behave.h 添加枚举值\n"
     "2. rob_act.h（或 rob_chs.h）声明 entry/exit/task 函数\n"
     "3. rob_act.c（或 rob_chs.c）实现函数\n"
     "4. rob_core.c 状态表中注册（注意枚举值和数组下标对应）\n"
     "5. rob_brain.c 添加触发条件"),
    ("修改硬件驱动的步骤",
     "修改 power_bsp.c 中的 Wheel_config / Track_config / Gantry_*_config 系列函数，\n"
     "或修改 rob_core.c 中 s_chs_power_cfg / s_act_power_cfg 的函数指针绑定。\n"
     "上层业务代码无需任何改动。"),
    ("调试技巧",
     "- 当前底盘状态：查看 RC_R2.Chs.now.id（对应 ChsState_e 枚举）\n"
     "- 当前动作状态：查看 RC_R2.Act.now.id（对应 ActEvent_e 枚举）\n"
     "- 模块私有变量以 s_ 前缀命名，仅在对应 .c 文件内可见，GDB 调试时可直接 watch\n"
     "- 任务通知位 kRobEvt_* 控制决策层的三个分支，可通过断点跟踪通知发送时机"),
]

for i, (title, desc) in enumerate(steps, 1):
    p = doc.add_paragraph()
    run = p.add_run(f"{i}. {title}")
    run.bold = True
    doc.add_paragraph(desc)

doc.add_page_break()

# ===================================================================
# 7. File List
# ===================================================================
doc.add_heading("七、文件清单", level=1)

table = doc.add_table(rows=8, cols=4, style="Light Grid Accent 1")
headers = ["文件", "所属层", "职责", "依赖"]
for i, h_text in enumerate(headers):
    cell = table.rows[0].cells[i]
    cell.text = h_text
    for p in cell.paragraphs:
        for r in p.runs:
            r.bold = True

data = [
    ["rob_behave.h", "Rob_core", "行为基类层：状态/动作结构体 + 枚举 + FreeRTOS 事件位", "main.h"],
    ["rob_power.h", "Rob_core", "动力接口层：底盘/上层机构的函数指针表", "main.h"],
    ["rob_core.h", "Rob_core", "核心封装层：派生类 + Robot_t + RC_R2 全局声明", "rob_behave.h, rob_power.h, rob_chs.h, rob_act.h"],
    ["rob_core.c", "Rob_core", "对象实例化：状态表/动力表配置 + RC_R2_Init", "rob_core.h"],
    ["rob_chs.h/c", "App", "底盘状态机：6 个状态的 entry/exit/task 实现", "rob_core.h"],
    ["rob_act.h/c", "App", "上层动作：5 个动作的 entry/exit/task 实现", "rob_core.h"],
    ["rob_brain.h/c", "App", "决策调度：StateFlag_Function 顶层状态机决策", "rob_chs.h, rob_act.h"],
]
for row_idx, row_data in enumerate(data):
    for col_idx, text in enumerate(row_data):
        table.rows[row_idx + 1].cells[col_idx].text = text

doc.add_paragraph()

# ===================================================================
# 8. Changelog
# ===================================================================
doc.add_heading("八、版本记录", level=1)

cl_table = doc.add_table(rows=4, cols=4, style="Light Grid Accent 1")
cl_headers = ["版本", "日期", "作者", "变更说明"]
for i, h_text in enumerate(cl_headers):
    cell = cl_table.rows[0].cells[i]
    cell.text = h_text
    for p in cell.paragraphs:
        for r in p.runs:
            r.bold = True

cl_data = [
    ["V2.1", "2026/04/29", "little.RC-R2", "重构为三层类型体系，分离 behave/power/core 头文件"],
    ["V2.0", "2026/04", "little.RC-R2", "引入 C++ 基类/派生类思想，FSM 状态机架构"],
    ["V1.0", "2026/03", "little.RC-R2", "初始版本，基础底盘和动作控制"],
]
for row_idx, row_data in enumerate(cl_data):
    for col_idx, text in enumerate(row_data):
        cl_table.rows[row_idx + 1].cells[col_idx].text = text

# ---- Save ----
output_path = r"C:\Users\zzx20\Desktop\RC-R2_代码架构文档.docx"
doc.save(output_path)
print(f"OK: {output_path}")
