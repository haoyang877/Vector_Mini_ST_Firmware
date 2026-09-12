"""Params page: online parameter table (full), Flash params and angle-error placeholders."""

from __future__ import annotations

import json
from pathlib import Path
import time
from typing import Any

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QAbstractItemView,
    QFileDialog,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPlainTextEdit,
    QProgressBar,
    QPushButton,
    QSplitter,
    QTabWidget,
    QTableWidget,
    QTableWidgetItem,
    QTableView,
    QTreeWidget,
    QTreeWidgetItem,
    QVBoxLayout,
    QWidget,
)

import pyqtgraph as pg

from mdrive_core.schema.flash_params import FIELDS, lut_angle_error_degrees, lut_raw_angle_degrees
from mdrive_core.schema.params import GROUPS, PARAMS

from mdrive_gui import theme
from mdrive_gui.app_state import AppState
from mdrive_gui.pages.params_model import (
    ACCESS_TEXT,
    COL_ACCESS,
    COL_CURRENT,
    COL_ID,
    COL_NAME,
    COL_NEW,
    COL_RANGE,
    COL_UNIT,
    ENCODING_TEXT,
    ParamTableModel,
    format_range,
)
from mdrive_gui.worker import DANGEROUS_WRITE_IDS, CanWorker


class ParamsPage(QWidget):
    """Tab 3: [在线参数] [Flash 参数] [角度误差曲线]."""

    def __init__(self, state: AppState, worker: CanWorker, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.state = state
        self.worker = worker
        self.model = ParamTableModel(self)
        self._flash_payload: dict[str, Any] | None = None
        self._lut_previous: list[float] | None = None

        self.sub_tabs = QTabWidget()
        self.sub_tabs.addTab(self._build_online_tab(), "在线参数")
        self.sub_tabs.addTab(self._build_flash_tab(), "Flash 参数")
        self.sub_tabs.addTab(self._build_angle_tab(), "角度误差曲线")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.addWidget(self.sub_tabs)

        worker.flashRead.connect(self._on_flash_read)
        worker.paramRead.connect(self._on_param_read)
        worker.paramsRead.connect(self._on_params_read)
        worker.progress.connect(self._on_progress)
        worker.finished.connect(self._on_finished)
        state.connectionChanged.connect(self._on_connection)
        self._on_connection(state.connected)

    # ---------------------------------------------------------- online tab
    def _build_online_tab(self) -> QWidget:
        page = QWidget()

        self.search_edit = QLineEdit()
        self.search_edit.setPlaceholderText("搜索名称/ID")
        self.search_edit.textChanged.connect(self._apply_filter)
        self.group_tree = QTreeWidget()
        self.group_tree.setHeaderHidden(True)
        self.group_tree.setMinimumWidth(190)
        self.group_tree.setMaximumWidth(240)
        counts: dict[str, int] = {group: 0 for group in GROUPS}
        for definition in PARAMS.values():
            counts[definition.group] += 1
        all_item = QTreeWidgetItem([f"全部 ({len(PARAMS)})"])
        all_item.setData(0, Qt.ItemDataRole.UserRole, None)
        self.group_tree.addTopLevelItem(all_item)
        for group in GROUPS:
            item = QTreeWidgetItem([f"{group} ({counts[group]})"])
            item.setData(0, Qt.ItemDataRole.UserRole, group)
            self.group_tree.addTopLevelItem(item)
        self.group_tree.setCurrentItem(all_item)
        self.group_tree.currentItemChanged.connect(self._apply_filter)

        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.addWidget(self.search_edit)
        left_layout.addWidget(self.group_tree)

        self.table = QTableView()
        self.table.setModel(self.model)
        self.table.setAlternatingRowColors(True)
        self.table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SelectionMode.ExtendedSelection)
        self.table.verticalHeader().setVisible(False)
        self.table.verticalHeader().setDefaultSectionSize(24)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setColumnWidth(COL_ID, 56)
        self.table.setColumnWidth(COL_NAME, 190)
        self.table.setColumnWidth(COL_CURRENT, 90)
        self.table.setColumnWidth(COL_UNIT, 60)
        self.table.setColumnWidth(COL_NEW, 90)
        self.table.setColumnWidth(COL_RANGE, 110)
        self.table.setColumnWidth(COL_ACCESS, 60)
        self.table.selectionModel().currentRowChanged.connect(self._show_detail)

        self.detail = QLabel("选中一行查看参数详情。")
        self.detail.setProperty("muted", True)
        self.detail.setWordWrap(True)
        self.detail.setMinimumHeight(42)

        self.read_selected_btn = QPushButton("读取选中")
        self.read_selected_btn.clicked.connect(self._read_selected)
        self.read_all_btn = QPushButton("读取全部")
        self.read_all_btn.clicked.connect(self._read_all)
        self.write_btn = QPushButton("写入选中")
        self.write_btn.setProperty("variant", "primary")
        self.write_btn.clicked.connect(self._write_selected)
        self.write_save_btn = QPushButton("写入并保存到设备")
        self.write_save_btn.setProperty("variant", "primary")
        self.write_save_btn.clicked.connect(self._write_and_save)
        self.import_btn = QPushButton("导入")
        self.import_btn.clicked.connect(self._import)
        self.export_btn = QPushButton("导出")
        self.export_btn.clicked.connect(self._export)
        self.ro_counter = QLabel()
        self.ro_counter.setProperty("muted", True)
        self.dirty_counter = QLabel()
        self.dirty_counter.setProperty("muted", True)
        self.progress = QProgressBar()
        self.progress.setVisible(False)
        self.progress.setMaximumWidth(220)

        op_row = QHBoxLayout()
        for button in (
            self.read_selected_btn,
            self.read_all_btn,
            self.write_btn,
            self.write_save_btn,
            self.import_btn,
            self.export_btn,
        ):
            op_row.addWidget(button)
        op_row.addWidget(self.progress)
        op_row.addStretch(1)
        op_row.addWidget(self.ro_counter)
        op_row.addWidget(self.dirty_counter)

        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.addWidget(self.table, 1)
        right_layout.addWidget(self.detail)
        right_layout.addLayout(op_row)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.addWidget(left)
        splitter.addWidget(right)
        splitter.setStretchFactor(1, 1)
        page_layout = QVBoxLayout(page)
        page_layout.addWidget(splitter)

        self.model.changedCountChanged.connect(self._update_counters)
        self._apply_filter()
        self._update_counters()
        return page

    # ------------------------------------------------------------ flash tab
    def _build_flash_tab(self) -> QWidget:
        page = QWidget()
        self.flash_info_label = QLabel(
            "尚未读取。点击“读取 Flash 参数”从设备读取完整记录（约 2.2 KB，含 1024 点编码器 LUT）。"
        )
        self.flash_info_label.setProperty("muted", True)
        self.flash_info_label.setWordWrap(True)

        self.flash_read_btn = QPushButton("读取 Flash 参数")
        self.flash_read_btn.setProperty("variant", "primary")
        self.flash_read_btn.clicked.connect(self._read_flash)
        self.flash_json_btn = QPushButton("导出 JSON")
        self.flash_json_btn.clicked.connect(lambda: self._export_flash("json"))
        self.flash_csv_btn = QPushButton("导出 CSV")
        self.flash_csv_btn.clicked.connect(lambda: self._export_flash("csv"))
        self.flash_blob_btn = QPushButton("原始 blob")
        self.flash_blob_btn.clicked.connect(lambda: self._export_flash("blob"))
        self.flash_angle_btn = QPushButton("查看角度误差曲线 →")
        self.flash_angle_btn.clicked.connect(lambda: self.sub_tabs.setCurrentIndex(2))

        op_row = QHBoxLayout()
        for button in (
            self.flash_read_btn,
            self.flash_json_btn,
            self.flash_csv_btn,
            self.flash_blob_btn,
            self.flash_angle_btn,
        ):
            op_row.addWidget(button)
        op_row.addStretch(1)

        self.flash_group = QTreeWidget()
        self.flash_group.setHeaderHidden(True)
        self.flash_group.setMinimumWidth(150)
        self.flash_group.setMaximumWidth(190)
        groups: list[str] = []
        for field in FIELDS:
            if field.group not in groups:
                groups.append(field.group)
        all_item = QTreeWidgetItem([f"全部 ({len(FIELDS)})"])
        all_item.setData(0, Qt.ItemDataRole.UserRole, None)
        self.flash_group.addTopLevelItem(all_item)
        for group in groups:
            count = sum(1 for field in FIELDS if field.group == group)
            item = QTreeWidgetItem([f"{group} ({count})"])
            item.setData(0, Qt.ItemDataRole.UserRole, group)
            self.flash_group.addTopLevelItem(item)
        self.flash_group.setCurrentItem(all_item)
        self.flash_group.currentItemChanged.connect(self._refresh_flash_table)

        self.flash_table = QTableWidget(0, 5)
        self.flash_table.setHorizontalHeaderLabels(["字段", "偏移", "类型", "值", "单位"])
        self.flash_table.verticalHeader().setVisible(False)
        self.flash_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.flash_table.horizontalHeader().setStretchLastSection(True)
        self.flash_table.setColumnWidth(0, 250)
        self.flash_table.setColumnWidth(1, 56)
        self.flash_table.setColumnWidth(2, 52)
        self.flash_table.setColumnWidth(3, 130)

        self.flash_hex = QPlainTextEdit()
        self.flash_hex.setReadOnly(True)
        self.flash_hex.setMaximumHeight(84)
        self.flash_hex.setProperty("mono", True)

        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.addWidget(self.flash_table, 1)
        right_layout.addWidget(self.flash_hex)
        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.addWidget(self.flash_group)
        splitter.addWidget(right)
        splitter.setStretchFactor(1, 1)

        layout = QVBoxLayout(page)
        layout.addLayout(op_row)
        layout.addWidget(self.flash_info_label)
        layout.addWidget(splitter, 1)
        return page

    # -------------------------------------------------------------- angle tab
    def _build_angle_tab(self) -> QWidget:
        page = QWidget()
        self.angle_read_btn = QPushButton("读取并绘制")
        self.angle_read_btn.setProperty("variant", "primary")
        self.angle_read_btn.clicked.connect(self._read_and_plot)
        self.angle_overlay_btn = QPushButton("叠加对比（保留当前曲线）")
        self.angle_overlay_btn.clicked.connect(self._keep_overlay)
        self.angle_png_btn = QPushButton("导出 PNG")
        self.angle_png_btn.clicked.connect(self._export_png)
        self.angle_csv_btn = QPushButton("导出 CSV")
        self.angle_csv_btn.clicked.connect(self._export_angle_csv)
        self.angle_stats = QLabel("尚未读取 LUT。公式：err_deg = LUT[i] × 360/65536（与固件 USB 导出一致）。")
        self.angle_stats.setProperty("muted", True)

        op_row = QHBoxLayout()
        for button in (self.angle_read_btn, self.angle_overlay_btn, self.angle_png_btn, self.angle_csv_btn):
            op_row.addWidget(button)
        op_row.addStretch(1)
        op_row.addWidget(self.angle_stats)

        self.angle_plot = pg.PlotWidget(background=theme.PLOT_BG)
        self.angle_plot.showGrid(x=True, y=True, alpha=0.25)
        self.angle_plot.setLabel("bottom", "原始角度 (°)")
        self.angle_plot.setLabel("left", "角度误差 (°)")
        self.angle_plot.addLegend(offset=(10, 10))

        layout = QVBoxLayout(page)
        layout.addLayout(op_row)
        layout.addWidget(self.angle_plot, 1)
        return page

    # --------------------------------------------------------- flash actions
    def _read_flash(self) -> None:
        if not self._require_connection():
            return
        self.progress.setVisible(True)
        self.progress.setRange(0, 100)
        self.progress.setValue(0)
        self.worker.submit("flash_read", node=self.state.selected_node)

    def _on_flash_read(self, payload: dict) -> None:
        self._flash_payload = payload
        info = payload["info"]
        self.flash_info_label.setText(
            f"magic 0x{info['magic']:08X} ✓ ｜ schema v{info['schema']} ｜ {info['size']} B ｜ "
            f"CRC32 0x{info['crc32']:08X} ✓ ｜ 读取于 {time.strftime('%H:%M:%S')}"
        )
        self._refresh_flash_table()
        if self.sub_tabs.currentIndex() == 2:
            self._plot_from_payload()

    def _refresh_flash_table(self) -> None:
        payload = self._flash_payload
        self.flash_table.setRowCount(0)
        if payload is None:
            return
        parsed = payload["parsed"]
        item = self.flash_group.currentItem()
        group = item.data(0, Qt.ItemDataRole.UserRole) if item else None
        for field in FIELDS:
            if group is not None and field.group != group:
                continue
            row = self.flash_table.rowCount()
            self.flash_table.insertRow(row)
            value = parsed.get(field.name)
            text = f"{value:.6g}" if isinstance(value, float) else str(value)
            for column, cell in enumerate((field.name, f"0x{field.offset:03X}", field.fmt, text, field.unit)):
                table_item = QTableWidgetItem(cell)
                if column in (1, 2, 3):
                    table_item.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
                self.flash_table.setItem(row, column, table_item)
        if group is None or group == "基础":
            profile = parsed.get("axis_profile", {})
            summary = f"magic=0x{profile.get('magic', 0):08X}"
            if profile.get("joint_type") is not None:
                summary += f", joint_type={profile.get('joint_type')}"
            if profile.get("legacy_name"):
                summary += f", name={profile.get('legacy_name')}"
            if not profile.get("configured", False):
                summary += ", 未配置"
            row = self.flash_table.rowCount()
            self.flash_table.insertRow(row)
            for column, cell in enumerate(("axis_profile", "0x8A0", "struct32", summary, "-")):
                self.flash_table.setItem(row, column, QTableWidgetItem(cell))
        if group is None or group == "编码器":
            lut_count = len(parsed.get("encoder_linearization_lut_q15", []))
            row = self.flash_table.rowCount()
            self.flash_table.insertRow(row)
            for column, cell in enumerate(
                ("encoder_linearization_lut_q15", "0x02C", "i16[1024]", f"{lut_count} 点（见“角度误差曲线”）", "q15")
            ):
                self.flash_table.setItem(row, column, QTableWidgetItem(cell))
        blob = payload["blob"]
        lines = []
        for offset in range(0, len(blob), 32):
            lines.append(f"{offset:04X}  {blob[offset:offset + 32].hex(' ')}")
        self.flash_hex.setPlainText("\n".join(lines))

    def _export_flash(self, kind: str) -> None:
        if self._flash_payload is None:
            self.state.log("WARN", "尚未读取 Flash 参数")
            return
        payload = self._flash_payload
        if kind == "blob":
            path, _ = QFileDialog.getSaveFileName(self, "导出原始记录", "flash_record.bin", "BIN (*.bin)")
            if not path:
                return
            Path(path).write_bytes(payload["blob"])
        elif kind == "json":
            path, _ = QFileDialog.getSaveFileName(self, "导出 Flash 参数", "flash_params.json", "JSON (*.json)")
            if not path:
                return
            data = {"info": payload["info"], "fields": payload["parsed"], "lut_stats": payload["lut"]}
            Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
        else:
            path, _ = QFileDialog.getSaveFileName(self, "导出 Flash 参数 CSV", "flash_params.csv", "CSV (*.csv)")
            if not path:
                return
            rows = ["field,offset,format,value,unit"]
            for field in FIELDS:
                rows.append(
                    f"{field.name},0x{field.offset:03X},{field.fmt},{payload['parsed'].get(field.name)},{field.unit}"
                )
            Path(path).write_text("\n".join(rows) + "\n", encoding="utf-8")
        self.state.log("INFO", f"已导出 {path}")

    # --------------------------------------------------------- angle actions
    def _read_and_plot(self) -> None:
        if not self._require_connection():
            return
        if self._flash_payload is None:
            self._read_flash()
        else:
            self._plot_from_payload()

    def _current_errors(self) -> list[float] | None:
        payload = self._flash_payload
        if payload is None:
            return None
        return lut_angle_error_degrees(payload["blob"])

    def _plot_from_payload(self) -> None:
        errors = self._current_errors()
        if errors is None:
            return
        stats = self._flash_payload["lut"]
        parsed = self._flash_payload["parsed"]
        self.angle_plot.clear()
        angles = lut_raw_angle_degrees()
        if self._lut_previous is not None:
            self.angle_plot.plot(
                angles, self._lut_previous, pen=pg.mkPen(theme.MUTED, width=1, style=Qt.PenStyle.DashLine), name="上次读取"
            )
        self.angle_plot.plot(angles, errors, pen=pg.mkPen(theme.ACCENT, width=2), name="本次读取")
        self.angle_stats.setText(
            f"点数 {stats['points']} ｜ 最大 {stats['max_deg']:.3f}° @ {stats['max_angle_deg']:.1f}° ｜ "
            f"RMS {stats['rms_deg']:.3f}° ｜ 峰峰 {stats['peak_to_peak_deg']:.3f}° ｜ "
            f"reverse={parsed['encoder_reverse']} ｜ calib={parsed['encoder_calib_flag']}"
        )
        self.state.log("INFO", "角度误差曲线已绘制（1024 点）")

    def _keep_overlay(self) -> None:
        errors = self._current_errors()
        if errors is None:
            self.state.log("WARN", "没有可保留的曲线，请先读取")
            return
        self._lut_previous = errors
        self.state.log("INFO", "已保留当前曲线为“上次读取”，下次绘制将叠加")

    def _export_png(self) -> None:
        if self._flash_payload is None:
            self.state.log("WARN", "尚未读取 LUT")
            return
        path, _ = QFileDialog.getSaveFileName(self, "导出曲线 PNG", "angle_error.png", "PNG (*.png)")
        if not path:
            return
        self.angle_plot.grab().save(path)
        self.state.log("INFO", f"曲线已导出到 {path}")

    def _export_angle_csv(self) -> None:
        errors = self._current_errors()
        if errors is None:
            self.state.log("WARN", "尚未读取 LUT")
            return
        path, _ = QFileDialog.getSaveFileName(self, "导出角度误差 CSV", "angle_error.csv", "CSV (*.csv)")
        if not path:
            return
        rows = ["index,raw_deg,err_deg"]
        for index, (angle, error) in enumerate(zip(lut_raw_angle_degrees(), errors)):
            rows.append(f"{index},{angle:.6f},{error:.6f}")
        Path(path).write_text("\n".join(rows) + "\n", encoding="utf-8")
        self.state.log("INFO", f"角度误差数据已导出到 {path}")

    # ------------------------------------------------------------- actions
    def _selected_ids(self) -> list[int]:
        rows = {index.row() for index in self.table.selectionModel().selectedRows()}
        return [self.model.param_at(row).id for row in sorted(rows)]

    def _require_connection(self) -> bool:
        if not self.state.connected:
            self.state.raise_error("CAN 设备未打开，请先在连接页打开设备")
            return False
        return True

    def _apply_filter(self) -> None:
        item = self.group_tree.currentItem()
        group = item.data(0, Qt.ItemDataRole.UserRole) if item else None
        self.model.set_filter(group, self.search_edit.text())
        self._update_counters()

    def _read_selected(self) -> None:
        if not self._require_connection():
            return
        ids = [identifier for identifier in self._selected_ids() if PARAMS[identifier].access != "cmd"]
        if not ids:
            self.state.log("WARN", "未选中可读取的参数行")
            return
        for identifier in ids:
            self.worker.submit("read_param", param_id=identifier, node=self.state.selected_node)

    def _read_all(self) -> None:
        if not self._require_connection():
            return
        self.progress.setVisible(True)
        self.progress.setRange(0, 100)
        self.progress.setValue(0)
        self.worker.submit("read_all", node=self.state.selected_node)

    def _confirm_dangerous(self, ids: list[int]) -> bool:
        dangerous = [PARAMS[identifier].name for identifier in ids if identifier in DANGEROUS_WRITE_IDS]
        if not dangerous:
            return True
        answer = QMessageBox.warning(
            self,
            "确认写入",
            f"以下命令会切换电机运行模式：\n\n{', '.join(dangerous)}\n\n确认继续写入？",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        return answer == QMessageBox.StandardButton.Yes

    def _write_selected(self) -> None:
        if not self._require_connection():
            return
        ids = self._selected_ids()
        items = self.model.pending_items(only_selected=ids)
        if not items:
            self.state.log("WARN", "选中行没有待写入的新值")
            return
        if not self._confirm_dangerous([int(item[0]) for item in items]):
            return
        self.worker.submit("write_params", items=items, node=self.state.selected_node)

    def _write_and_save(self) -> None:
        if not self._require_connection():
            return
        items = self.model.pending_items()
        answer = QMessageBox.warning(
            self,
            "确认保存到 Flash",
            f"将写入 {len(items)} 项参数并发送 0x68 SAVE_PARAM 持久化到 Flash（擦写寿命有限）。\n\n确认继续？",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if answer != QMessageBox.StandardButton.Yes:
            return
        if not self._confirm_dangerous([int(item[0]) for item in items]):
            return
        self.worker.submit("write_and_save", items=items, node=self.state.selected_node)

    def _export(self) -> None:
        snapshot = self.model.values_snapshot()
        if not snapshot:
            self.state.log("WARN", "没有已读取的参数值可导出")
            return
        path, _ = QFileDialog.getSaveFileName(self, "导出参数", "params.json", "JSON (*.json)")
        if not path:
            return
        payload = {
            "node": self.state.selected_node,
            "values": {f"0x{identifier:02X}": value for identifier, value in sorted(snapshot.items())},
        }
        try:
            Path(path).write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        except OSError as exc:
            self.state.raise_error(f"导出参数失败: {exc}")
            return
        self.state.log("INFO", f"参数已导出到 {path}")

    def _import(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "导入参数", "", "JSON (*.json)")
        if not path:
            return
        try:
            payload = json.loads(Path(path).read_text(encoding="utf-8"))
            values = payload["values"] if isinstance(payload, dict) and "values" in payload else payload
        except (OSError, ValueError, KeyError) as exc:
            self.state.raise_error(f"导入参数失败: {exc}")
            return
        applied = 0
        for key, value in values.items():
            try:
                identifier = int(str(key), 0)
                number = float(value)
            except (ValueError, TypeError):
                continue
            if self.model.set_pending(identifier, number):
                applied += 1
        self.state.log("INFO", f"已导入 {applied} 项待写入参数（导入项进入“新值”列，需写入后生效）")

    # --------------------------------------------------------------- events
    def _on_param_read(self, param_id: int, value: Any) -> None:
        self.model.set_value(param_id, value)

    def _on_params_read(self, values: dict) -> None:
        self.model.set_values({int(key): val for key, val in values.items()})
        self.state.set_telemetry(values.get(0x2D), values.get(0x2F), values.get(0x43))

    def _on_progress(self, task: str, done: int, total: int) -> None:
        if task not in ("read_all", "flash_read") or total <= 0:
            return
        self.progress.setVisible(done < total)
        self.progress.setValue(int(done * 100 / total))

    def _on_finished(self, task: str, payload: dict) -> None:
        if task in ("read_all", "flash_read"):
            self.progress.setVisible(False)
        elif task == "write_params":
            for identifier in payload.get("results", {}):
                self.model.clear_pending(int(identifier))
        elif task == "save_params":
            self.state.log("INFO", "已保存到设备 Flash")

    def _on_connection(self, connected: bool) -> None:
        for button in (
            self.read_selected_btn,
            self.read_all_btn,
            self.write_btn,
            self.write_save_btn,
            self.flash_read_btn,
            self.angle_read_btn,
        ):
            button.setEnabled(connected)

    def _update_counters(self) -> None:
        self.ro_counter.setText(f"只读项 {self.model.readonly_count()}")
        count = self.model.pending_count()
        self.dirty_counter.setText(f"未保存更改 {count}")
        self.dirty_counter.setStyleSheet(f"color: {theme.WARN}" if count else f"color: {theme.MUTED}")

    def _show_detail(self, current) -> None:
        if not current.isValid():
            return
        definition = self.model.param_at(current.row())
        parts = [
            f"0x{definition.id:02X} {definition.name}",
            f"分组 {definition.group}",
            f"编码 {ENCODING_TEXT[definition.encoding]}",
            f"单位 {definition.unit or '—'}",
            f"范围 {format_range(definition)}",
            f"权限 {ACCESS_TEXT[definition.access]}",
        ]
        if definition.get_id is not None:
            parts.append(f"写后校验：读 0x{definition.get_id:02X} 比对")
        if definition.note:
            parts.append(f"备注：{definition.note}")
        if definition.id in DANGEROUS_WRITE_IDS:
            parts.append("提示：写该命令会切换电机模式，需二次确认")
        self.detail.setText(" ｜ ".join(parts))
