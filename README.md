# PCB 钢网 + 治具 生成器

把 PCB 设计文件直接变成**能拿去 3D 打印的钢网（stencil）和治具（jig）**。

输入用 Gerber RS-274X + Excellon 钻孔——Altium、立创EDA、KiCad 等所有 EDA 都能导出，
是事实上的通用格式。输出 STL（打印用）和 STEP（给 CAD/CNC 用）。

## 直接用（推荐）

下载 `dist/钢网治具生成器.exe`（或到 Release 页面下载），双击打开：

- 选板子目录（或 zip），点 **① 预览** 看效果，点 **② 生成并导出** 出文件
- 左边是俯视图（蓝=钢网、深灰=治具、黄虚线=板框、红=打穿孔禁开区），右边是剖面图
- 参数随便改：钢网厚度、板厚、治具壁厚、间隙、开孔补偿……改完回车即时刷新

免装 Python，免装任何依赖，拷到车间电脑就能用。

## 命令行（批量）

```bat
钢网治具生成器.exe -i <板子目录或zip> -o <输出目录> [参数]
```

常用参数：

| 参数 | 默认 | 说明 |
|---|---|---|
| `--layer` | `top` | 用顶层还是底层钢网（`top`/`bottom`） |
| `--stencil` | `0.15` | 钢网厚度 mm |
| `--board` | `1.6` | 板厚 mm（决定治具空腔深度） |
| `--jig-wall` | `5.0` | 治具壁厚 mm |
| `--clearance` | `0.20` | 板框与治具的间隙 mm |
| `--hole-clearance` | `0.15` | 打穿孔禁布区外扩 mm |
| `--hole-policy` | `clip` | 打穿孔处理：`clip` 只挖掉孔位、焊盘保留 / `drop` 整个焊盘删掉 |
| `--via-dia` | `0.40` | ≤该直径的孔算过孔，不参与避让 mm |
| `--min-aperture` | `0.15` | 小于此宽度的开孔丢弃 mm |
| `--offset` | `0.0` | 开孔补偿（正=放大）mm |
| `--no-jig` / `--no-flip` / `--no-html` | | 不要治具 / 不翻转打印姿态 / 不出 3D 预览 |

```bat
:: 例：底层钢网、0.12 厚、补偿 0.05
钢网治具生成器.exe -i D:\板子\工程目录 -o D:\输出 --layer bottom --stencil 0.12 --offset 0.05
```

自检（不接真实文件，跑内置合成样例）：

```bat
钢网治具生成器.exe --selftest        :: 命令行自检
钢网治具生成器.exe --gui-check       :: 界面自检（自动开窗跑一遍再关）
```

## 设计要点

1. **排孔，不排焊盘**——用钻孔文件生成禁布区，和钢网开孔做布尔运算，只把**孔位**挖掉，
   焊盘本身留着（`clip`，默认）。2.54 排针、安装孔这类打穿孔不会被开成钢网，而
   **盘中孔**（焊盘里穿了散热过孔）的焊盘会完整保留：直径 ≤ `--via-dia`（默认 0.40）
   的孔算过孔，根本不进禁布区。焊盘整个压在孔上、挖完只剩印不出来的细边时，宁可
   整个焊盘原样保留并在日志里点名（这几处锡膏会印到孔上）；想连焊盘一起删就用 `drop`。
2. **总有一个面是平的**——治具顶面与钢网顶面严格共面，导出时整体翻转 180° 让钢网面贴热床，
   全程无悬空、无桥接，3D 打印机好打。
3. **网格必须是闭合实体**——带孔多边形用 GEOS *约束* Delaunay 三角化（尊重输入边、不添
   Steiner 点），保证顶面三角化和侧壁顶点集完全一致，不会出现 T 型接点导致的破面。
   每次生成都会自检并报告：破面边数、非流形边数、体积与理论值偏差。

## 源码运行 / 重新打包

源码只有一个文件 `function.py`（自带 Gerber/Excellon 解析、布尔运算、三角化、
STL/STEP 写出、tkinter 界面，不依赖 OpenCASCADE）。

```bat
conda activate mess_3.11
pip install shapely
python function.py                   :: 开界面
python function.py --selftest        :: 自检
```

打包成 exe：

```bat
pip install pyinstaller
set BIN=D:\APPS\ENV\envs\mess_3.11\Library\bin
pyinstaller --noconfirm --onefile --console --name 钢网治具生成器 ^
    --distpath dist --workpath build --specpath build ^
    --add-binary "%BIN%\tcl86t.dll;." --add-binary "%BIN%\tk86t.dll;." ^
    --add-binary "%BIN%\zlib1.dll;." --add-binary "%BIN%\ffi-8.dll;." ^
    function.py
```

> 那四个 `--add-binary` 不能省：conda 把 tcl/tk 的 dll 放在 `Library\bin`，
> PyInstaller 扫不到，漏了的话打出来的 exe 一开界面就 `DLL load failed`。

## 输出文件

| 文件 | 用途 |
|---|---|
| `stencil_t<厚>_b<板厚>.stl` | 直接丢进切片软件打印 |
| `stencil_t<厚>_b<板厚>.step` | 给 CAD / CNC 用（AP214，单个闭合实体） |
| `stencil_t<厚>_b<板厚>_preview.html` | 浏览器里转着看的 3D 预览，双击即开 |
