"""
Mask Editor - Odin1鱼眼相机SLAM Mask编辑工具
用于创建和编辑SLAM定位的mask图片，屏蔽相机视场中的固定物体。
"""

import sys
import os
import numpy as np
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QGraphicsView, QGraphicsScene,
    QGraphicsPixmapItem, QToolBar, QStatusBar, QFileDialog,
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QButtonGroup,
    QLabel, QRadioButton, QGroupBox, QMessageBox, QGraphicsEllipseItem,
    QGraphicsLineItem, QGraphicsRectItem, QGraphicsPolygonItem,
    QSlider
)
from PyQt6.QtGui import (
    QImage, QPixmap, QPainter, QColor, QPen, QBrush, QAction,
    QKeySequence, QPolygonF, QCursor, QWheelEvent
)
from PyQt6.QtCore import (
    Qt, QPointF, QRectF, pyqtSignal, QObject
)


# ============================================================
# MaskModel - mask数据模型 + undo/redo
# ============================================================
class MaskModel(QObject):
    """管理mask图片数据和撤销/重做栈"""
    mask_changed = pyqtSignal()

    MASK_W = 1600
    MASK_H = 1296
    MAX_UNDO = 30

    def __init__(self):
        super().__init__()
        self._mask = QImage(self.MASK_W, self.MASK_H, QImage.Format.Format_Grayscale8)
        self._mask.fill(QColor(255, 255, 255))
        self._undo_stack = []
        self._redo_stack = []

    def get_mask(self):
        return self._mask

    def create_blank(self):
        self._mask.fill(QColor(255, 255, 255))
        self._undo_stack.clear()
        self._redo_stack.clear()
        self.mask_changed.emit()

    def load_mask(self, path):
        img = QImage(path)
        if img.isNull():
            return False
        img = img.scaled(self.MASK_W, self.MASK_H)
        img = img.convertToFormat(QImage.Format.Format_Grayscale8)
        self._mask = img
        self._undo_stack.clear()
        self._redo_stack.clear()
        self.mask_changed.emit()
        return True

    def save_mask(self, path):
        return self._mask.save(path, "PNG")

    def begin_stroke(self):
        """在操作前保存当前状态"""
        self._undo_stack.append(self._mask.copy())
        if len(self._undo_stack) > self.MAX_UNDO:
            self._undo_stack.pop(0)
        self._redo_stack.clear()

    def undo(self):
        if self._undo_stack:
            self._redo_stack.append(self._mask.copy())
            self._mask = self._undo_stack.pop()
            self.mask_changed.emit()

    def redo(self):
        if self._redo_stack:
            self._undo_stack.append(self._mask.copy())
            self._mask = self._redo_stack.pop()
            self.mask_changed.emit()

    def paint_at(self, x, y, radius, color=0):
        """在指定位置画圆"""
        painter = QPainter(self._mask)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, False)
        c = QColor(color, color, color)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QBrush(c))
        painter.drawEllipse(QPointF(x, y), radius, radius)
        painter.end()
        self.mask_changed.emit()

    def fill_rect(self, rect, color=0):
        painter = QPainter(self._mask)
        c = QColor(color, color, color)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QBrush(c))
        painter.drawRect(rect)
        painter.end()
        self.mask_changed.emit()

    def fill_ellipse(self, rect, color=0):
        painter = QPainter(self._mask)
        c = QColor(color, color, color)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QBrush(c))
        painter.drawEllipse(rect)
        painter.end()
        self.mask_changed.emit()

    def fill_polygon(self, points, color=0):
        painter = QPainter(self._mask)
        c = QColor(color, color, color)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QBrush(c))
        polygon = QPolygonF([QPointF(p[0], p[1]) for p in points])
        painter.drawPolygon(polygon)
        painter.end()
        self.mask_changed.emit()


# ============================================================
# Drawing Tools
# ============================================================
class ToolBase:
    """绘图工具基类"""
    def __init__(self, model, canvas):
        self.model = model
        self.canvas = canvas

    def on_press(self, pos):
        pass

    def on_move(self, pos):
        pass

    def on_release(self, pos):
        pass

    def on_double_click(self, pos):
        pass

    def cancel(self):
        pass

    def get_color(self):
        return 0  # black


class BrushTool(ToolBase):
    """画笔工具"""
    SIZES = {'S': 10, 'M': 30, 'L': 60}

    def __init__(self, model, canvas):
        super().__init__(model, canvas)
        self.size_key = 'M'
        self._drawing = False
        self._last_pos = None

    @property
    def radius(self):
        return self.SIZES[self.size_key]

    def on_press(self, pos):
        self.model.begin_stroke()
        self._drawing = True
        self._last_pos = pos
        self.model.paint_at(pos.x(), pos.y(), self.radius, self.get_color())

    def on_move(self, pos):
        if not self._drawing:
            return
        # 插值避免间隙
        if self._last_pos:
            dx = pos.x() - self._last_pos.x()
            dy = pos.y() - self._last_pos.y()
            dist = (dx * dx + dy * dy) ** 0.5
            step = max(self.radius / 3, 2)
            if dist > step:
                steps = int(dist / step)
                for i in range(1, steps + 1):
                    t = i / steps
                    ix = self._last_pos.x() + dx * t
                    iy = self._last_pos.y() + dy * t
                    self.model.paint_at(ix, iy, self.radius, self.get_color())
        self.model.paint_at(pos.x(), pos.y(), self.radius, self.get_color())
        self._last_pos = pos

    def on_release(self, pos):
        self._drawing = False
        self._last_pos = None


class EraserTool(BrushTool):
    """橡皮工具 - 涂白色"""
    def get_color(self):
        return 255


class RectTool(ToolBase):
    """矩形工具"""
    def __init__(self, model, canvas):
        super().__init__(model, canvas)
        self._start = None
        self._preview = None

    def on_press(self, pos):
        self.model.begin_stroke()
        self._start = pos
        self._preview = self.canvas.scene().addRect(
            QRectF(pos, pos),
            QPen(QColor(255, 0, 0), 2, Qt.PenStyle.DashLine)
        )

    def on_move(self, pos):
        if self._start and self._preview:
            rect = QRectF(self._start, pos).normalized()
            self._preview.setRect(rect)

    def on_release(self, pos):
        if self._start:
            rect = QRectF(self._start, pos).normalized()
            self.model.fill_rect(rect, self.get_color())
        if self._preview:
            self.canvas.scene().removeItem(self._preview)
            self._preview = None
        self._start = None


class EllipseTool(ToolBase):
    """椭圆工具"""
    def __init__(self, model, canvas):
        super().__init__(model, canvas)
        self._start = None
        self._preview = None

    def on_press(self, pos):
        self.model.begin_stroke()
        self._start = pos
        self._preview = self.canvas.scene().addEllipse(
            QRectF(pos, pos),
            QPen(QColor(255, 0, 0), 2, Qt.PenStyle.DashLine)
        )

    def on_move(self, pos):
        if self._start and self._preview:
            rect = QRectF(self._start, pos).normalized()
            self._preview.setRect(rect)

    def on_release(self, pos):
        if self._start:
            rect = QRectF(self._start, pos).normalized()
            self.model.fill_ellipse(rect, self.get_color())
        if self._preview:
            self.canvas.scene().removeItem(self._preview)
            self._preview = None
        self._start = None


class PolygonTool(ToolBase):
    """多边形工具 - 单击添加顶点，双击闭合填充"""
    def __init__(self, model, canvas):
        super().__init__(model, canvas)
        self._points = []
        self._preview_items = []
        self._stroke_begun = False

    def on_press(self, pos):
        if not self._stroke_begun:
            self.model.begin_stroke()
            self._stroke_begun = True

        self._points.append((pos.x(), pos.y()))

        # 画顶点标记
        marker = self.canvas.scene().addEllipse(
            pos.x() - 4, pos.y() - 4, 8, 8,
            QPen(QColor(255, 0, 0), 1),
            QBrush(QColor(255, 0, 0))
        )
        self._preview_items.append(marker)

        # 画连线
        if len(self._points) > 1:
            p1 = self._points[-2]
            p2 = self._points[-1]
            line = self.canvas.scene().addLine(
                p1[0], p1[1], p2[0], p2[1],
                QPen(QColor(255, 0, 0), 2, Qt.PenStyle.DashLine)
            )
            self._preview_items.append(line)

    def on_double_click(self, pos):
        if len(self._points) >= 3:
            self.model.fill_polygon(self._points, self.get_color())
        self._clear_preview()

    def cancel(self):
        self._clear_preview()

    def _clear_preview(self):
        for item in self._preview_items:
            self.canvas.scene().removeItem(item)
        self._preview_items.clear()
        self._points.clear()
        self._stroke_begun = False


# ============================================================
# MaskCanvas - 画布
# ============================================================
class MaskCanvas(QGraphicsView):
    """画布: 支持缩放/平移, 层叠显示mask和话题图片"""
    coords_changed = pyqtSignal(int, int)

    def __init__(self, model):
        super().__init__()
        self.model = model
        self._scene = QGraphicsScene()
        self.setScene(self._scene)

        # 场景层
        self._image_item = QGraphicsPixmapItem()
        self._mask_item = QGraphicsPixmapItem()
        self._scene.addItem(self._image_item)
        self._scene.addItem(self._mask_item)

        # 设置
        self.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform, False)
        self.setTransformationAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOn)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOn)

        # 状态
        self._active_tool = None
        self._edit_mode = True  # True=编辑模式, False=检查模式
        self._panning = False
        self._pan_start = None
        self._topic_image = None
        self._mask_opacity = 0.45  # 编辑模式下mask层透明度(可调)

        # 连接信号
        self.model.mask_changed.connect(self._refresh_mask_display)
        self._refresh_mask_display()

    def set_tool(self, tool):
        if self._active_tool:
            self._active_tool.cancel()
        self._active_tool = tool

    def set_edit_mode(self, edit_mode):
        self._edit_mode = edit_mode
        self._refresh_mask_display()

    def set_mask_opacity(self, value):
        """设置编辑模式下mask层透明度 (0-100)"""
        self._mask_opacity = value / 100.0
        self._refresh_mask_display()

    def set_topic_image(self, qimage):
        """设置话题图片"""
        self._topic_image = qimage
        if qimage and not qimage.isNull():
            self._image_item.setPixmap(QPixmap.fromImage(qimage))
        else:
            self._image_item.setPixmap(QPixmap())
        self._refresh_mask_display()

    def _refresh_mask_display(self):
        """刷新mask显示层"""
        mask = self.model.get_mask()
        if self._edit_mode:
            # 编辑模式: 图片全显示, mask层半透明覆盖(透明度可调)
            self._image_item.setOpacity(1.0)
            self._mask_item.setPixmap(QPixmap.fromImage(mask))
            self._mask_item.setOpacity(self._mask_opacity)
        else:
            # 检查模式: 话题图片上叠加半透明红色mask
            self._image_item.setOpacity(1.0)
            if self._topic_image and not self._topic_image.isNull():
                overlay = self._create_overlay(mask)
                self._mask_item.setPixmap(QPixmap.fromImage(overlay))
                self._mask_item.setOpacity(1.0)
            else:
                self._mask_item.setPixmap(QPixmap.fromImage(mask))
                self._mask_item.setOpacity(0.5)

    def _create_overlay(self, mask):
        """创建检查模式的半透明覆盖层"""
        w, h = mask.width(), mask.height()
        overlay = QImage(w, h, QImage.Format.Format_ARGB32)
        overlay.fill(QColor(0, 0, 0, 0))

        # 用numpy加速
        mask_ptr = mask.bits()
        if mask_ptr is None:
            return overlay
        mask_arr = np.frombuffer(mask_ptr.asstring(w * h), dtype=np.uint8).reshape(h, w)

        overlay_ptr = overlay.bits()
        if overlay_ptr is None:
            return overlay
        overlay_arr = np.frombuffer(
            overlay_ptr.asstring(w * h * 4), dtype=np.uint8
        ).reshape(h, w, 4)

        # 注意: QImage ARGB32格式在内存中是 BGRA
        # mask=0(黑色) → 覆盖红色半透明
        masked_pixels = mask_arr == 0
        overlay_arr_copy = overlay_arr.copy()
        overlay_arr_copy[masked_pixels, 0] = 0     # B
        overlay_arr_copy[masked_pixels, 1] = 0     # G
        overlay_arr_copy[masked_pixels, 2] = 200   # R
        overlay_arr_copy[masked_pixels, 3] = 120   # A

        # 写回
        result = QImage(overlay_arr_copy.data, w, h, w * 4, QImage.Format.Format_ARGB32)
        return result.copy()  # copy确保data不被释放

    def fit_view(self):
        self.fitInView(self._scene.sceneRect(), Qt.AspectRatioMode.KeepAspectRatio)

    # --- 鼠标事件 ---
    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.MiddleButton:
            self._panning = True
            self._pan_start = event.position().toPoint()
            self.setCursor(QCursor(Qt.CursorShape.ClosedHandCursor))
            return

        if event.button() == Qt.MouseButton.LeftButton and self._active_tool and self._edit_mode:
            pos = self.mapToScene(event.position().toPoint())
            self._active_tool.on_press(pos)
            return

        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        scene_pos = self.mapToScene(event.position().toPoint())
        self.coords_changed.emit(int(scene_pos.x()), int(scene_pos.y()))

        if self._panning and self._pan_start:
            delta = event.position().toPoint() - self._pan_start
            self._pan_start = event.position().toPoint()
            self.horizontalScrollBar().setValue(
                self.horizontalScrollBar().value() - delta.x()
            )
            self.verticalScrollBar().setValue(
                self.verticalScrollBar().value() - delta.y()
            )
            return

        if event.buttons() & Qt.MouseButton.LeftButton and self._active_tool and self._edit_mode:
            pos = self.mapToScene(event.position().toPoint())
            self._active_tool.on_move(pos)
            return

        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.MiddleButton:
            self._panning = False
            self.setCursor(QCursor(Qt.CursorShape.ArrowCursor))
            return

        if event.button() == Qt.MouseButton.LeftButton and self._active_tool and self._edit_mode:
            pos = self.mapToScene(event.position().toPoint())
            self._active_tool.on_release(pos)
            return

        super().mouseReleaseEvent(event)

    def mouseDoubleClickEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton and self._active_tool and self._edit_mode:
            pos = self.mapToScene(event.position().toPoint())
            self._active_tool.on_double_click(pos)
            return
        super().mouseDoubleClickEvent(event)

    def wheelEvent(self, event):
        if event.modifiers() & Qt.KeyboardModifier.ControlModifier:
            factor = 1.15 if event.angleDelta().y() > 0 else 1 / 1.15
            self.scale(factor, factor)
        else:
            super().wheelEvent(event)


# ============================================================
# ImageBrowser - 图片浏览管理
# ============================================================
class ImageBrowser:
    """管理导入的话题图片"""
    def __init__(self):
        self._folder = ""
        self._files = []
        self._index = 0

    def set_folder(self, path):
        self._folder = path
        exts = ('.jpg', '.jpeg', '.png', '.bmp')
        self._files = sorted([
            f for f in os.listdir(path)
            if f.lower().endswith(exts)
        ])
        self._index = 0

    def count(self):
        return len(self._files)

    def current_index(self):
        return self._index

    def current_image(self):
        if not self._files:
            return None
        path = os.path.join(self._folder, self._files[self._index])
        img = QImage(path)
        if img.isNull():
            return None
        return img

    def next(self):
        if self._files and self._index < len(self._files) - 1:
            self._index += 1
            return True
        return False

    def previous(self):
        if self._files and self._index > 0:
            self._index -= 1
            return True
        return False


# ============================================================
# MaskEditorWindow - 主窗口
# ============================================================
class MaskEditorWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Mask Editor - Odin1 SLAM")
        self.resize(1200, 900)

        # 数据模型
        self.model = MaskModel()
        self.browser = ImageBrowser()

        # 画布
        self.canvas = MaskCanvas(self.model)

        # 工具
        self._tools = {}
        self._current_tool_name = 'brush'
        self._brush_size = 'M'
        self._init_tools()

        # UI
        self._setup_toolbar()
        self._setup_menubar()
        self._setup_statusbar()
        self._setup_layout()

        # 设置初始工具
        self._switch_tool('brush')
        self.canvas.fit_view()

    def _init_tools(self):
        self._tools = {
            'brush': BrushTool(self.model, self.canvas),
            'eraser': EraserTool(self.model, self.canvas),
            'rect': RectTool(self.model, self.canvas),
            'ellipse': EllipseTool(self.model, self.canvas),
            'polygon': PolygonTool(self.model, self.canvas),
        }

    def _setup_toolbar(self):
        """左侧工具栏"""
        self.toolbar_widget = QWidget()
        layout = QVBoxLayout(self.toolbar_widget)
        layout.setContentsMargins(4, 4, 4, 4)

        # 工具按钮
        tools_group = QGroupBox("工具")
        tools_layout = QVBoxLayout(tools_group)

        self.tool_buttons = QButtonGroup(self)
        tool_defs = [
            ('brush', '画笔 (B)'),
            ('eraser', '橡皮 (E)'),
            ('rect', '矩形 (R)'),
            ('ellipse', '椭圆 (C)'),
            ('polygon', '多边形 (P)'),
        ]
        for name, label in tool_defs:
            btn = QPushButton(label)
            btn.setCheckable(True)
            btn.setFixedHeight(32)
            self.tool_buttons.addButton(btn)
            btn.clicked.connect(lambda checked, n=name: self._switch_tool(n))
            tools_layout.addWidget(btn)
            if name == 'brush':
                btn.setChecked(True)

        layout.addWidget(tools_group)

        # 笔刷大小
        size_group = QGroupBox("笔刷大小")
        size_layout = QVBoxLayout(size_group)
        self.size_buttons = QButtonGroup(self)

        for key, label in [('S', '小 (1)'), ('M', '中 (2)'), ('L', '大 (3)')]:
            btn = QRadioButton(label)
            self.size_buttons.addButton(btn)
            btn.clicked.connect(lambda checked, k=key: self._set_brush_size(k))
            size_layout.addWidget(btn)
            if key == 'M':
                btn.setChecked(True)

        layout.addWidget(size_group)

        # 模式切换
        mode_group = QGroupBox("模式")
        mode_layout = QVBoxLayout(mode_group)
        self.edit_mode_btn = QRadioButton("编辑模式")
        self.check_mode_btn = QRadioButton("检查模式")
        self.edit_mode_btn.setChecked(True)
        self.edit_mode_btn.clicked.connect(lambda: self._set_mode(True))
        self.check_mode_btn.clicked.connect(lambda: self._set_mode(False))
        mode_layout.addWidget(self.edit_mode_btn)
        mode_layout.addWidget(self.check_mode_btn)

        # Mask透明度滑块
        self.opacity_label = QLabel("Mask透明度: 45%")
        mode_layout.addWidget(self.opacity_label)
        self.opacity_slider = QSlider(Qt.Orientation.Horizontal)
        self.opacity_slider.setRange(10, 90)
        self.opacity_slider.setValue(45)
        self.opacity_slider.setTickInterval(10)
        self.opacity_slider.valueChanged.connect(self._on_opacity_changed)
        mode_layout.addWidget(self.opacity_slider)

        layout.addWidget(mode_group)

        # 图片导航
        nav_group = QGroupBox("图片导航")
        nav_layout = QVBoxLayout(nav_group)
        self.nav_label = QLabel("无图片")
        self.nav_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        nav_layout.addWidget(self.nav_label)
        nav_btn_layout = QHBoxLayout()
        self.prev_btn = QPushButton("◀ 上一张")
        self.next_btn = QPushButton("下一张 ▶")
        self.prev_btn.clicked.connect(self._prev_image)
        self.next_btn.clicked.connect(self._next_image)
        nav_btn_layout.addWidget(self.prev_btn)
        nav_btn_layout.addWidget(self.next_btn)
        nav_layout.addLayout(nav_btn_layout)
        layout.addWidget(nav_group)

        # 操作按钮
        ops_group = QGroupBox("操作")
        ops_layout = QVBoxLayout(ops_group)

        import_btn = QPushButton("导入图片文件夹")
        import_btn.clicked.connect(self._import_images)
        ops_layout.addWidget(import_btn)

        load_btn = QPushButton("加载已有Mask")
        load_btn.clicked.connect(self._load_mask)
        ops_layout.addWidget(load_btn)

        export_btn = QPushButton("导出Mask")
        export_btn.clicked.connect(self._export_mask)
        ops_layout.addWidget(export_btn)

        new_btn = QPushButton("新建空白Mask")
        new_btn.clicked.connect(self._new_mask)
        ops_layout.addWidget(new_btn)

        layout.addWidget(ops_group)
        layout.addStretch()

        self.toolbar_widget.setFixedWidth(170)

    def _setup_menubar(self):
        menubar = self.menuBar()

        # 文件菜单
        file_menu = menubar.addMenu("文件(&F)")

        new_action = QAction("新建Mask", self)
        new_action.setShortcut(QKeySequence("Ctrl+N"))
        new_action.triggered.connect(self._new_mask)
        file_menu.addAction(new_action)

        load_action = QAction("加载Mask", self)
        load_action.setShortcut(QKeySequence("Ctrl+O"))
        load_action.triggered.connect(self._load_mask)
        file_menu.addAction(load_action)

        import_action = QAction("导入图片文件夹", self)
        import_action.setShortcut(QKeySequence("Ctrl+I"))
        import_action.triggered.connect(self._import_images)
        file_menu.addAction(import_action)

        export_action = QAction("导出Mask", self)
        export_action.setShortcut(QKeySequence("Ctrl+E"))
        export_action.triggered.connect(self._export_mask)
        file_menu.addAction(export_action)

        file_menu.addSeparator()
        exit_action = QAction("退出", self)
        exit_action.setShortcut(QKeySequence("Ctrl+Q"))
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)

        # 编辑菜单
        edit_menu = menubar.addMenu("编辑(&E)")

        undo_action = QAction("撤销", self)
        undo_action.setShortcut(QKeySequence("Ctrl+Z"))
        undo_action.triggered.connect(self.model.undo)
        edit_menu.addAction(undo_action)

        redo_action = QAction("重做", self)
        redo_action.setShortcut(QKeySequence("Ctrl+Y"))
        redo_action.triggered.connect(self.model.redo)
        edit_menu.addAction(redo_action)

        # 视图菜单
        view_menu = menubar.addMenu("视图(&V)")

        fit_action = QAction("适应窗口", self)
        fit_action.setShortcut(QKeySequence("Ctrl+0"))
        fit_action.triggered.connect(self.canvas.fit_view)
        view_menu.addAction(fit_action)

    def _setup_statusbar(self):
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_label = QLabel("就绪")
        self.image_label = QLabel("无图片")
        self.coord_label = QLabel("(0, 0)")
        self.mode_label = QLabel("[编辑模式]")
        self.status_bar.addWidget(self.mode_label)
        self.status_bar.addWidget(self.image_label)
        self.status_bar.addPermanentWidget(self.coord_label)

        self.canvas.coords_changed.connect(self._update_coords)

    def _setup_layout(self):
        central = QWidget()
        layout = QHBoxLayout(central)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.toolbar_widget)
        layout.addWidget(self.canvas, 1)
        self.setCentralWidget(central)

    # --- 工具/模式切换 ---
    def _switch_tool(self, name):
        self._current_tool_name = name
        tool = self._tools[name]
        if name in ('brush', 'eraser'):
            tool.size_key = self._brush_size
        self.canvas.set_tool(tool)

    def _set_brush_size(self, key):
        self._brush_size = key
        for name in ('brush', 'eraser'):
            self._tools[name].size_key = key

    def _set_mode(self, edit_mode):
        self.canvas.set_edit_mode(edit_mode)
        if edit_mode:
            self.mode_label.setText("[编辑模式]")
            self.edit_mode_btn.setChecked(True)
        else:
            self.mode_label.setText("[检查模式]")
            self.check_mode_btn.setChecked(True)

    # --- 文件操作 ---
    def _import_images(self):
        folder = QFileDialog.getExistingDirectory(self, "选择图片文件夹")
        if folder:
            self.browser.set_folder(folder)
            self._show_current_image()
            self.status_label.setText(f"已导入: {folder}")

    def _show_current_image(self):
        img = self.browser.current_image()
        self.canvas.set_topic_image(img)
        count = self.browser.count()
        idx = self.browser.current_index()
        if count > 0:
            text = f"{idx + 1}/{count}"
            self.image_label.setText(f"图片: {text}")
            self.nav_label.setText(text)
        else:
            self.image_label.setText("无图片")
            self.nav_label.setText("无图片")

    def _load_mask(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "加载Mask文件", "", "图片文件 (*.png *.jpg *.bmp)"
        )
        if path:
            if self.model.load_mask(path):
                self.status_label.setText(f"已加载Mask: {path}")
            else:
                QMessageBox.warning(self, "错误", f"无法加载: {path}")

    def _export_mask(self):
        path, _ = QFileDialog.getSaveFileName(
            self, "导出Mask文件", "mask.png", "PNG图片 (*.png)"
        )
        if path:
            if self.model.save_mask(path):
                QMessageBox.information(self, "成功", f"Mask已导出到:\n{path}")
            else:
                QMessageBox.warning(self, "错误", f"导出失败: {path}")

    def _new_mask(self):
        reply = QMessageBox.question(
            self, "新建Mask", "确定创建新的空白Mask？当前编辑将丢失。"
        )
        if reply == QMessageBox.StandardButton.Yes:
            self.model.create_blank()

    def _update_coords(self, x, y):
        self.coord_label.setText(f"({x}, {y})")

    def _on_opacity_changed(self, value):
        """透明度滑块变化"""
        self.opacity_label.setText(f"Mask透明度: {value}%")
        self.canvas.set_mask_opacity(value)

    def _prev_image(self):
        """切换到上一张图片"""
        if self.browser.previous():
            self._show_current_image()

    def _next_image(self):
        """切换到下一张图片"""
        if self.browser.next():
            self._show_current_image()

    # --- 快捷键 ---
    def keyPressEvent(self, event):
        key = event.key()
        mod = event.modifiers()

        # 左右切换图片
        if key == Qt.Key.Key_Left:
            if self.browser.previous():
                self._show_current_image()
            return
        if key == Qt.Key.Key_Right:
            if self.browser.next():
                self._show_current_image()
            return

        # Tab切换模式
        if key == Qt.Key.Key_Tab:
            is_edit = self.canvas._edit_mode
            self._set_mode(not is_edit)
            return

        # 工具切换
        if key == Qt.Key.Key_B:
            self._switch_tool('brush')
            return
        if key == Qt.Key.Key_E:
            self._switch_tool('eraser')
            return
        if key == Qt.Key.Key_R:
            self._switch_tool('rect')
            return
        if key == Qt.Key.Key_C:
            self._switch_tool('ellipse')
            return
        if key == Qt.Key.Key_P:
            self._switch_tool('polygon')
            return

        # 笔刷大小
        if key == Qt.Key.Key_1:
            self._set_brush_size('S')
            return
        if key == Qt.Key.Key_2:
            self._set_brush_size('M')
            return
        if key == Qt.Key.Key_3:
            self._set_brush_size('L')
            return

        # Escape取消当前操作
        if key == Qt.Key.Key_Escape:
            if self._active_tool:
                self._active_tool = self._tools[self._current_tool_name]
                self._active_tool.cancel()
            return

        super().keyPressEvent(event)


# ============================================================
# Main
# ============================================================
def main():
    app = QApplication(sys.argv)
    app.setApplicationName("Mask Editor")

    window = MaskEditorWindow()
    window.show()

    # 初始适应窗口
    window.canvas.fit_view()

    sys.exit(app.exec())


if __name__ == '__main__':
    main()
