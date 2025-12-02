import sys
import os
import re
import yaml
from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QTableWidget, QTableWidgetItem, QLabel, QMessageBox,
    QDoubleSpinBox, QGroupBox, QFormLayout, QHeaderView
)
from PySide6.QtCore import Qt

# ---------- 配置与常量 ----------
CONFIG_PATH = "src/fairino_bringup/config/fairino_control/custom_points_config.yaml"
LAUNCH_FILE = "launch.sh"

# 柜子点位 (Block Scalar String)
KEY_FRONT = "cabinet_front_points"
KEY_INSIDE = "cabinet_inside_points"
KEY_RECOG = "cabinet_recognize_points"
POINT_KEYS = [KEY_FRONT, KEY_INSIDE, KEY_RECOG]

# 静态列表点位 (Simple List [x, y, z])
KEY_ABOVE_CAR = "item_above_car_point"
KEY_IN_CAR = "item_in_car_point"
LIST_POINT_KEYS = [KEY_ABOVE_CAR, KEY_IN_CAR]


# ---------- 文件处理工具 ----------

def read_file_text(path):
    with open(path, "r", encoding="utf-8") as f:
        return f.read()

def write_file_text(path, text):
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)

def parse_block_list_from_string(block_string):
    """解析 block string 为 列表"""
    lines = block_string.splitlines()
    out = []
    for ln in lines:
        s = ln.strip()
        if not s: continue
        # 匹配 - [1, 2, 3] 或 - 1, 2, 3
        m = re.match(r"^-\s*\[?(.*?)\]?\s*$", s)
        if m:
            inner = m.group(1).strip()
            try:
                # 尝试利用 yaml 解析内部数组
                arr = yaml.safe_load("[" + inner + "]")
                if isinstance(arr, list) and len(arr) >= 3:
                    out.append([float(x) for x in arr])
            except:
                pass
    return out

def build_block_string_from_list(list_of_points, indent="      "):
    """将列表转回 block string"""
    lines = []
    for p in list_of_points:
        # 格式化：整数显示为整数，浮点保留小数
        vals = ", ".join(str(int(x)) if float(x).is_integer() else str(x) for x in p)
        lines.append(f"{indent}- [{vals}]")
    return "\n".join(lines) + "\n"

def extract_block_for_key(full_text, key):
    """提取 key 对应的 block (针对 Block Scalar 格式)"""
    pattern = rf"(^[ \t]*{re.escape(key)}\s*:\s*\|\s*\n)((?:[ \t]+-.*\n)*)"
    m = re.search(pattern, full_text, flags=re.MULTILINE)
    if not m:
        return None, None, None
    prefix = m.group(1)
    block = m.group(2)
    return prefix, block, m.span()

def replace_block_for_key(full_text, key, new_block_content):
    """替换指定 Key 的 Block 内容"""
    prefix, old_block, span = extract_block_for_key(full_text, key)
    if prefix is None:
        return full_text
    
    start, end = span
    new_text = full_text[:start] + prefix + new_block_content + full_text[end:]
    return new_text

def detect_all_points(full_text):
    """解析所有 Block Scalar 点位"""
    result = {}
    for k in POINT_KEYS:
        prefix, block, span = extract_block_for_key(full_text, k)
        if prefix:
            result[k] = parse_block_list_from_string(block)
        else:
            result[k] = []
    return result

# -----------------------------------------------------------------
# 针对简单列表参数 (item_above_car_point, item_in_car_point) 的工具函数
# -----------------------------------------------------------------

def read_list_parameter_from_text(key, text):
    """使用正则表达式从文本中读取 key: [x, y, z] 的列表值"""
    # 匹配 key: [x, y, z] (x, y, z 可以包含空格和逗号)
    pattern = rf"^[ \t]*{re.escape(key)}\s*:\s*\[(.*?)\]"
    m = re.search(pattern, text, flags=re.MULTILINE)
    val = [0.0, 0.0, 0.0]
    
    if m:
        content = m.group(1).strip()
        if content:
            try:
                # 按逗号分割并转浮点
                parts = content.split(',')
                temp_list = [float(p.strip()) for p in parts]
                if len(temp_list) >= 3:
                    val = temp_list[:3]
            except:
                pass 
    return val

def write_list_parameter_to_text(key, values, text):
    """使用正则表达式将 key: [x, y, z] 的列表值写回文本"""
    
    # 如果值是整数，格式化为 .1f (例如 5.0)，如果不是整数，则保留其原有字符串精度。
    def format_to_double_string(x):
        x_float = float(x)
        if x_float.is_integer():
            # 格式化为至少保留一位小数的浮点字符串 (例如 5 -> 5.0)
            return f"{x_float:.1f}"
        else:
            # 非整数保留原有精度 (例如 5.123 -> 5.123)
            return str(x_float)

    # 构造 [x, y, z] 字符串
    vals_str = ", ".join(format_to_double_string(x) for x in values)
    
    # 匹配 key: [...] 的整行 (保留前后的缩进和换行符)
    cp_pattern = rf"(^[ \t]*{re.escape(key)}\s*:\s*\[)(.*)(\].*)"
    m_cp = re.search(cp_pattern, text, flags=re.MULTILINE)
    
    if m_cp:
        # 替换中间的数值部分，保留前缀([)和后缀(])
        new_line = f"{m_cp.group(1)}{vals_str}{m_cp.group(3)}"
        # 使用切片和拼接替换整行内容，保留了原始缩进和换行
        text = text[:m_cp.start()] + new_line + text[m_cp.end():]
        
    return text


# ---------- GUI 主程序 ----------

class PointStringEditor(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS2 点位配置编辑器 (Final Fixed)")
        self.resize(950, 700) # 调整窗口大小

        self.config_path = CONFIG_PATH
        
        # 1. 读取文件与解析
        try:
            self.full_text = read_file_text(self.config_path)
        except Exception as e:
            QMessageBox.critical(self, "错误", f"无法读取配置文件：\n{e}")
            self.full_text = ""

        self.points_data = detect_all_points(self.full_text)
        
        # 2. 计算初始 Offset
        init_y_off, init_z_off = self.calculate_initial_offsets()

        # --- UI 布局 ---
        main_layout = QVBoxLayout()

        # A. 静态点位 (只保留 Item Points)
        static_group = QGroupBox("1. 静态点位编辑 (item_above_car_point, item_in_car_point)")
        static_layout = QVBoxLayout()
        
        # item_above_car_point
        self.table_above = self._create_point_table()
        self._load_list_point_to_table(KEY_ABOVE_CAR, self.table_above)
        static_layout.addWidget(QLabel(f"<b>{KEY_ABOVE_CAR}</b>:"))
        static_layout.addWidget(self.table_above)

        # item_in_car_point
        self.table_in = self._create_point_table()
        self._load_list_point_to_table(KEY_IN_CAR, self.table_in)
        static_layout.addWidget(QLabel(f"<b>{KEY_IN_CAR}</b>:"))
        static_layout.addWidget(self.table_in)
        
        static_group.setLayout(static_layout)
        main_layout.addWidget(static_group)


        # B. 偏移量设置
        offset_group = QGroupBox("2. 自动生成规则参数")
        form_layout = QFormLayout()
        
        self.spin_offset_y = QDoubleSpinBox()
        self.spin_offset_y.setRange(-2000, 2000)
        self.spin_offset_y.setValue(init_y_off)
        self.spin_offset_y.setSuffix(" (Inside Y = Front Y + 此值)")

        self.spin_offset_z = QDoubleSpinBox()
        self.spin_offset_z.setRange(-2000, 2000)
        self.spin_offset_z.setValue(init_z_off)
        self.spin_offset_z.setSuffix(" (Recognize Z = Front Z - 此值)")

        form_layout.addRow("柜内 Y轴增量:", self.spin_offset_y)
        form_layout.addRow("识别 Z轴减量:", self.spin_offset_z)
        offset_group.setLayout(form_layout)
        main_layout.addWidget(offset_group)

        # C. 核心编辑区
        front_group = QGroupBox(f"3. 柜前点位编辑 ({KEY_FRONT})")
        front_layout = QVBoxLayout()
        front_layout.addWidget(QLabel("说明：修改此表，保存时会自动更新 '柜内' 和 '识别' 点位。"))
        
        self.table_front = QTableWidget()
        self.table_front.setColumnCount(3)
        self.table_front.setHorizontalHeaderLabels(["X", "Y", "Z"])
        self.table_front.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.load_front_points()
        front_layout.addWidget(self.table_front)
        front_group.setLayout(front_layout)
        main_layout.addWidget(front_group)

        # D. 按钮
        btn_layout = QHBoxLayout()
        btn_save = QPushButton("保存所有配置")
        btn_save.setStyleSheet("background-color: #4CAF50; color: white; font-size: 14px; padding: 8px;")
        btn_save.clicked.connect(self.on_save)
        
        btn_reload = QPushButton("重置/重新加载")
        btn_reload.clicked.connect(self.on_reload)
        
        btn_launch = QPushButton("运行 launch.sh")
        btn_launch.clicked.connect(self.run_launch)

        btn_layout.addWidget(btn_save)
        btn_layout.addWidget(btn_reload)
        btn_layout.addWidget(btn_launch)
        main_layout.addLayout(btn_layout)

        self.setLayout(main_layout)

    # ---------- 辅助创建函数 (属于类方法) ----------

    def _create_point_table(self):
        """创建并初始化用于 [x, y, z] 的单行表格"""
        table = QTableWidget()
        table.setColumnCount(3)
        table.setRowCount(1)
        table.setHorizontalHeaderLabels(["X", "Y", "Z"])
        table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        table.setFixedHeight(70)
        return table

    def _load_list_point_to_table(self, key, table):
        """通用函数：从文本中读取 [x, y, z] 列表并填充到表格"""
        val = read_list_parameter_from_text(key, self.full_text)
        for j in range(3):
            table.setItem(0, j, QTableWidgetItem(str(val[j])))

    def _read_list_point_from_table(self, table):
        """通用函数：从表格中读取 [x, y, z] 列表"""
        vals = []
        for j in range(3):
            it = table.item(0, j)
            try:
                # 尝试读取并转浮点数
                vals.append(float(it.text()) if it and it.text().strip() else 0.0)
            except:
                vals.append(0.0)
        return vals

    # ---------- 逻辑处理 ----------

    def calculate_initial_offsets(self):
        """对比现有点位，反推增量"""
        front = self.points_data.get(KEY_FRONT, [])
        inside = self.points_data.get(KEY_INSIDE, [])
        recog = self.points_data.get(KEY_RECOG, [])

        def_y = 50.0
        def_z = 20.0

        if front and inside and len(front) > 0 and len(inside) > 0:
            def_y = inside[0][1] - front[0][1]
        
        if front and recog and len(front) > 0 and len(recog) > 0:
            def_z = front[0][2] - recog[0][2]

        return def_y, def_z

    def load_front_points(self):
        pts = self.points_data.get(KEY_FRONT, [])
        self.table_front.setRowCount(len(pts))
        for i, p in enumerate(pts):
            for j in range(3):
                val = p[j] if j < len(p) else 0.0
                self.table_front.setItem(i, j, QTableWidgetItem(str(val)))

    def on_reload(self):
        try:
            self.full_text = read_file_text(self.config_path)
            self.points_data = detect_all_points(self.full_text)
            
            ny, nz = self.calculate_initial_offsets()
            self.spin_offset_y.setValue(ny)
            self.spin_offset_z.setValue(nz)
            
            # 重新加载 Item 列表点位
            self._load_list_point_to_table(KEY_ABOVE_CAR, self.table_above)
            self._load_list_point_to_table(KEY_IN_CAR, self.table_in)
            
            self.load_front_points()
            QMessageBox.information(self, "提示", "已重新读取文件")
        except Exception as e:
            QMessageBox.critical(self, "错误", str(e))

    def on_save(self):
        # 1. Offset
        off_y = self.spin_offset_y.value()
        off_z = self.spin_offset_z.value()

        # 2. Front Data (核心点位)
        rows = self.table_front.rowCount()
        front_pts = []
        for i in range(rows):
            row_data = []
            for j in range(3):
                it = self.table_front.item(i, j)
                txt = it.text() if it else "0"
                try:
                    row_data.append(float(txt))
                except:
                    row_data.append(0.0)
            front_pts.append(row_data)

        # 3. Auto generate
        inside_pts = []
        recog_pts = []
        for p in front_pts:
            inside_pts.append([p[0], p[1] + off_y, p[2]])
            recog_pts.append([p[0], p[1], p[2] - off_z])

        # 4. 替换 Block Scalar 点位
        write_map = {
            KEY_FRONT: front_pts,
            KEY_INSIDE: inside_pts,
            KEY_RECOG: recog_pts
        }

        new_text = self.full_text

        for key, pts_list in write_map.items():
            prefix, old_block, _ = extract_block_for_key(new_text, key)
            indent = "      "
            if old_block:
                lines = old_block.splitlines()
                for l in lines:
                    m_ind = re.match(r"^([ \t]+)-", l)
                    if m_ind:
                        indent = m_ind.group(1)
                        break
            
            new_block = build_block_string_from_list(pts_list, indent)
            new_text = replace_block_for_key(new_text, key, new_block)

        # 5. 替换静态列表点位 (Item Points)
        
        # Item Above Car Point
        above_vals = self._read_list_point_from_table(self.table_above)
        new_text = write_list_parameter_to_text(KEY_ABOVE_CAR, above_vals, new_text)

        # Item In Car Point
        in_vals = self._read_list_point_from_table(self.table_in)
        new_text = write_list_parameter_to_text(KEY_IN_CAR, in_vals, new_text)

        # 6. Write
        try:
            write_file_text(self.config_path, new_text)
            self.full_text = new_text
            self.points_data = detect_all_points(self.full_text)
            self.calculate_initial_offsets()
            QMessageBox.information(self, "成功", "配置已保存！(静态点位已格式化为浮点数)")
        except Exception as e:
            QMessageBox.critical(self, "保存失败", str(e))

    def run_launch(self):
        if not os.path.exists(LAUNCH_FILE):
            QMessageBox.critical(self, "错误", f"找不到 {LAUNCH_FILE}")
            return
        # 使用 os.system 启动一个新终端执行脚本
        os.system(f"gnome-terminal -- bash -c 'cd {os.getcwd()}; chmod +x {LAUNCH_FILE}; ./{LAUNCH_FILE}; exec bash'")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = PointStringEditor()
    w.show()
    sys.exit(app.exec())