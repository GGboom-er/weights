# coding:utf-8
import os.path

try:
    from PySide.QtGui import *
    from PySide.QtCore import *
except ImportError:
    from PySide2.QtGui import *
    from PySide2.QtCore import *
    from PySide2.QtWidgets import *

from . import explorer
from . import weight_tool
from . import sculpt_weight
from . import bezier_tool


labels = [u"次级", u"关节", u"脊椎",  u"背带", u"眉毛", u"多点", u"多线", u"多链", u"手指"]
names = ["soft", "ik", "spine", "belt", "brow", "points", "lines", "chains", "finger"]


class Bezier(QWidget):
    valueChanged = Signal()

    def __init__(self):
        QWidget.__init__(self)
        self.points = [[0.0, 1.0], [1.0 / 3, 1.0], [2.0 / 3, 0.0], [1.0, 0.0]]
        self.__movePoint = 0
        self.__mirror = False
        self.__adsorb = False
        self.setFixedSize(512, 448)

    def paintEvent(self, event):
        QWidget.paintEvent(self, event)
        painter = QPainter(self)
        # background
        painter.setBrush(QBrush(QColor(120, 120, 120), Qt.SolidPattern))
        painter.setPen(QPen(QColor(0, 0, 0), 1, Qt.SolidLine))
        painter.drawRect(0, 0, self.width()-1, self.height()-1)
        # curve
        painter.setBrush(QBrush(QColor(100, 100, 100), Qt.SolidPattern))
        points = [QPointF((self.width()-1) * p[0], (self.height()-1) * p[1]) for p in self.points]
        path = QPainterPath()
        path.moveTo(0, self.height()-1)
        path.lineTo(points[0])
        path.cubicTo(*points[1:])
        path.lineTo(self.width()-1, self.height()-1)
        painter.drawPath(path)
        # grid
        painter.setPen(QPen(QColor(200, 200, 200), 1, Qt.DotLine))
        w_step = (self.width()-1)/6.0
        h_step = (self.height()-1)/6.0
        for i in range(1, 6):
            w = w_step * i
            h = h_step * i
            painter.drawLine(w, 0, w, self.height())
            painter.drawLine(0, h, self.width(), h)
        # control point
        painter.setPen(QPen(QColor(0, 0, 0), 1, Qt.SolidLine))
        painter.setBrush(QBrush(QColor(200, 200, 200), Qt.SolidPattern))
        painter.drawEllipse(points[1], 6, 6)
        painter.drawEllipse(points[2], 6, 6)
        # edge
        painter.setPen(QPen(QColor(0, 0, 0), 1, Qt.SolidLine))
        edge_points = []
        for w, h in zip([0, 0, 1, 1, 0], [0, 1, 1, 0, 0]):
            p = QPointF(w*(self.width()-1), h*(self.height()-1))
            edge_points.extend([p, p])
        painter.drawLines(edge_points[1:-1])
        # control line
        painter.setPen(QPen(QColor(200, 200, 200), 1, Qt.DashLine))
        painter.drawLine(points[0], points[1])
        painter.drawLine(points[3], points[2])
        painter.end()

    def mousePressEvent(self, event):
        self.setFocus()
        QWidget.mousePressEvent(self, event)
        points = [QPointF((self.width() - 1) * p[0], (self.height() - 1) * p[1]) for p in self.points]
        p = QPointF(event.pos())-points[1]
        length = (p.x()**2 + p.y()**2)**0.5
        if length < 10:
            self.__movePoint = 1
            return
        p = QPointF(event.pos()) - points[2]
        length = (p.x() ** 2 + p.y() ** 2) ** 0.5
        if length < 10:
            self.__movePoint = 2
            return
        self.__movePoint = 0

    def mouseMoveEvent(self, event):
        QWidget.mouseMoveEvent(self, event)
        if self.__movePoint == 1:
            p = QPointF(event.pos())
            x = max(min(float(p.x())/(self.width()-1), 1.0), 0.0)
            y = max(min(float(p.y())/(self.height()-1), 1.0), 0.0)
            if self.__adsorb:
                x = round(x*12)/12.0
                y = round(y*12)/12.0
            if self.__mirror:
                mx = (1-x)
                my = (1-y)
                self.points[2] = [mx, my]
            self.points[1] = [x, y]
            self.update()
            self.valueChanged.emit()
        if self.__movePoint == 2:
            p = QPointF(event.pos())
            x = max(min(float(p.x())/(self.width()-1), 1.0), 0.0)
            y = max(min(float(p.y())/(self.height()-1), 1.0), 0.0)
            if self.__adsorb:
                x = round(x*6)/6.0
                y = round(y*6)/6.0
            if self.__mirror:
                mx = (1-x)
                my = (1-y)
                self.points[1] = [mx, my]
            self.points[2] = [x, y]
            self.update()
            self.valueChanged.emit()

    def keyPressEvent(self, event):
        QWidget.keyPressEvent(self, event)
        if event.key() == Qt.Key_X:
            self.__adsorb = True
        if event.modifiers() == Qt.ControlModifier:
            self.__mirror = True

    def keyReleaseEvent(self, event):
        QWidget.keyReleaseEvent(self, event)
        self.__mirror = False
        self.__adsorb = False


class FloatSliderGroup(QHBoxLayout):
    valueChange = Signal(float)

    def __init__(self, prefix=u""):
        QHBoxLayout.__init__(self)
        if prefix:
            self.addWidget(QLabel(prefix))
        self.slider = QSlider(Qt.Horizontal)
        self.spin = QDoubleSpinBox()
        self.addWidget(self.spin)
        self.addWidget(self.slider)
        self.set_range(0, 1)
        self.spin.valueChanged.connect(self.convert_slider)
        self.slider.valueChanged.connect(self.convert_spin)
        self.spin.setSingleStep(0.01)
        self.spin.setDecimals(3)

    def convert_spin(self, value):
        self.spin.setValue(value/1000.0)
        self.valueChange.emit(self.spin.value())
        self.spin.setMinimumWidth(65)

    def convert_slider(self, value):
        self.slider.setValue(int(round(1000*value)))

    def set_range(self, min_value, max_value):
        self.spin.setRange(min_value, max_value)
        self.slider.setRange(min_value*1000, max_value*1000)

    def value(self):
        return self.spin.value()

    def set_value(self, value):
        self.spin.setValue(value)


class IntSliderGroup(QHBoxLayout):
    valueChange = Signal(float)

    def __init__(self, prefix=u""):
        QHBoxLayout.__init__(self)
        if prefix:
            self.addWidget(QLabel(prefix))
        self.slider = QSlider(Qt.Horizontal)
        self.spin = QSpinBox()
        self.addWidget(self.spin)
        self.addWidget(self.slider)
        self.spin.valueChanged.connect(self.slider.setValue)
        self.slider.valueChanged.connect(self.spin.setValue)
        self.spin.setMinimumWidth(65)

    def set_range(self, min_value, max_value):
        self.spin.setRange(min_value, max_value)
        self.slider.setRange(min_value, max_value)

    def value(self):
        return self.spin.value()

    def set_value(self, value):
        self.spin.setValue(value)


class MaxInfluence(QDialog):

    def __init__(self, parent):
        QDialog.__init__(self, parent)
        self.setWindowTitle(u"限制最大影响数")
        layout = QVBoxLayout()
        self.setLayout(layout)
        self.slider = IntSliderGroup(u"最大影响数:")
        self.slider.set_range(1, 64)
        layout.addLayout(self.slider)

        button = QPushButton(u"限制最大影响数")
        button.clicked.connect(self.apply)
        layout.addWidget(button)

        kwargs = weight_tool.get_json_kwargs(weight_tool.limit_max_influence)
        max_influence = kwargs.get("max_influence", 4)
        self.slider.set_value(max_influence)
        self.resize(300, 10)

    def apply(self):
        max_influence = self.slider.value()
        weight_tool.limit_max_influence(max_influence=max_influence)


class LockInfluenceSmooth(QDialog):

    def __init__(self, parent):
        QDialog.__init__(self, parent)
        self.setWindowTitle(u"锁定影响平滑")
        layout = QVBoxLayout()

        self.setLayout(layout)
        self.count = IntSliderGroup(u"平滑次数：")
        self.count.set_range(1, 100)
        layout.addLayout(self.count)

        self.step = FloatSliderGroup(u"平滑强度：")
        self.step.set_range(0, 1)
        layout.addLayout(self.step)

        button = QPushButton(u"锁定影响平滑")
        button.clicked.connect(self.apply)
        layout.addWidget(button)

        kwargs = weight_tool.get_json_kwargs(weight_tool.lock_influence_smooth)
        smooth_count = kwargs.get("smooth_count", 1)
        smooth_step = kwargs.get("smooth_step", 1)
        self.count.set_value(smooth_count)
        self.step.set_value(smooth_step)
        self.resize(300, 10)

    def apply(self):
        smooth_count = self.count.value()
        smooth_step = self.step.value()
        weight_tool.lock_influence_smooth(smooth_count=smooth_count, smooth_step=smooth_step)


def test():
    window = LockInfluenceSmooth(get_host_app())
    window.show()





qss = u"""
QWidget{
    font-size: 16px;
    font-family: 楷体;
} 
QToolBar{
    border-top:1px solid #242424;
    border-bottom:1px solid #242424;
    background:  #4B4B4B;
    padding: 4px;
}

QToolButton{
    background:  #5B5B5B;
    border-radius:8px;
    font-size: 12px;
    margin: 4px;
    padding-top: 4px;
    padding-bottom: 4px;
}
QToolButton::hover{
    background:  #6B6B6B;
}
QToolButton::pressed{
    background:  #5B5B5B;
}
"""


def get_host_app():
    try:
        main_window = QApplication.activeWindow()
        while True:
            last_win = main_window.parent()
            if last_win:
                main_window = last_win
            else:
                break
        return main_window
    except:
        pass


class BezierWeight(QDialog):

    def __init__(self):
        QDialog.__init__(self, get_host_app())
        self.setWindowTitle("bezierWeight")
        self.setStyleSheet(qss)

        layout = QVBoxLayout()
        self.setLayout(layout)
        menu_bar = QMenuBar()

        layout.setMenuBar(menu_bar)
        tool_bar = QToolBar()
        tool_bar.setIconSize(QSize(32, 32))
        tool_bar.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
        layout.addWidget(tool_bar)
        icon_path = os.path.abspath(__file__+"/../icons/").replace("\\", "/")
        tool_bar.addAction(QIcon(icon_path+"/copyWeights.png"), u"拷贝权重", weight_tool.copy_weights)
        tool_bar.addAction(QIcon(icon_path+"/resetWeight.png"), u"重置蒙皮", weight_tool.re_skin)
        tool_bar.addAction(QIcon(icon_path + "/cacheWeight.png"), u"缓存权重", weight_tool.copy_joint_weight)
        tool_bar.addAction(QIcon(icon_path + "/loadWeight.png"), u"加载权重", weight_tool.paste_joint_weight)
        tool_bar.addAction(QIcon(icon_path + "/lockInfluence.png"), u"限制影响", weight_tool.limit_max_influence)
        tool_bar.addAction(QIcon(icon_path + "/smoothWeight.png"), u"平滑权重", weight_tool.lock_influence_smooth)
        tool_bar.addAction(QIcon(icon_path + "/sculptWeight.png"), u"雕刻权重", sculpt_weight.auto_apply)

        self.max_influence = MaxInfluence(self)
        self.lock_smooth = LockInfluenceSmooth(self)
        menu = menu_bar.addMenu(u"工具")
        menu.addAction(u"创建定位器", weight_tool.create_vx)
        menu.addAction(u"计算眼皮权重", weight_tool.paint_eye)
        menu.addAction(u"复制蒙皮层", weight_tool.copy_unlock_skin_polygon)
        menu.addAction(u"平滑权重", self.lock_smooth.show)
        menu.addAction(u"限制影响", self.max_influence.show)

        menu = menu_bar.addMenu(u"帮助")
        menu.addAction(u"使用说明", explorer.help)
        menu.addAction(u"C++极速版下载", explorer.download)
        menu.addAction(u"开发教程", explorer.study)

        layout.setContentsMargins(1, 1, 1, 1)
        self.bezier = Bezier()
        layout.addWidget(self.bezier)
        layout.addStretch()
        h_layout = QHBoxLayout()
        layout.addLayout(h_layout)
        h_layout.addStretch()
        argument_form = QFormLayout()
        h_layout.addLayout(argument_form)
        h_layout.addStretch()

        self.type = QButtonGroup()
        radio_layout = QGridLayout()
        for i, text in enumerate(labels):
            radio = QRadioButton(text)
            self.type.addButton(radio, i)
            radio_layout.addWidget(radio, i / 3, i % 3, 1, 1)
        self.type.button(0).setChecked(True)
        argument_form.addRow(u"类型：", radio_layout)

        self.mode = QCheckBox()
        argument_form.addRow(u"实时：", self.mode)

        self.radius = FloatSliderGroup()
        self.radius.set_range(0, 2)
        self.radius.set_value(1)
        argument_form.addRow(u"半径：", self.radius)

        button = QPushButton(u"解算")
        layout.addStretch()
        layout.addWidget(button)

        self.type.buttonClicked.connect(self.type_changed)
        button.clicked.connect(self.paint_weight)
        self.bezier.valueChanged.connect(self.real_paint_weight)
        self.radius.valueChange.connect(self.real_paint_weight)
        self.mode.stateChanged.connect(self.real_mode_changed)
        self.on_real = False

    def type_changed(self):
        self.mode.setChecked(Qt.Unchecked)

    def real_mode_changed(self):
        self.on_real = False
        if self.mode.isChecked():
            self.on_real = bezier_tool.init(names[self.type.checkedId()])
            self.real_paint_weight()

    def get_ui_kwargs(self):
        typ = names[self.type.checkedId()]
        r = self.radius.value()
        xs = [x for x, y in self.bezier.points]
        ys = [1 - y for x, y in self.bezier.points]
        return typ, xs, ys, r

    def paint_weight(self):
        if self.mode.isChecked():
            self.real_paint_weight()
        else:
            bezier_tool.paint(*self.get_ui_kwargs())

    def real_paint_weight(self):
        if self.on_real:
            bezier_tool.solve(*self.get_ui_kwargs())


if __name__ == '__main__':
    app = QApplication([])
    window = BezierWeight()
    window.show()
    app.exec_()




window = None


def show():
    global window
    if window is None:
        window = BezierWeight()
    window.show()

