import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget,QIcon,QGraphicsScene,QImage,QGraphicsPixmapItem,QPixmap,QPen,QBrush,QGraphicsView
from python_qt_binding.QtCore import Qt,qWarning,QRectF,QSize,pyqtSignal

from sensor_msgs.msg import Image
from robostylus_planner.msg import Point2D

class RQT_NDSView(Plugin):

    _qEllipseObjects = []

    _pub_points = None

    _sub_top = None
    _sub_bottom = None
    _sub_top_debug = None
    _sub_bottom_debug = None

    _signal = pyqtSignal(Image, QGraphicsPixmapItem, QGraphicsView)

    _qGraphicsSceneTop = QGraphicsScene()
    _qGraphicsSceneBottom = QGraphicsScene()
    _qGraphicsSceneTopDebug = QGraphicsScene()
    _qGraphicsSceneBottomDebug = QGraphicsScene()

    _qPixmapItemTop = QGraphicsPixmapItem()
    _qPixmapItemBottom = QGraphicsPixmapItem()
    _qPixmapItemTopDebug = QGraphicsPixmapItem()
    _qPixmapItemBottomDebug = QGraphicsPixmapItem()

    def __init__(self, context):
        super(RQT_NDSView, self).__init__(context)

        # Give QObjects reasonable names
        self.setObjectName('RQT_NDSView')

        # Create QWidget
        self._widget = QWidget()

        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('robostylus_gui'), 'resource', 'rqt_ndsview.ui')

        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)

        # Give QObjects reasonable names
        self._widget.setObjectName('RQT_NDSView_UI')
        self._widget.setWindowTitle('NDS View')

        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget).
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)

        # Configure individual widgets
        self._widget.checkBox_debug.stateChanged.connect(self.on_checkBox_debug_changed)

        self._signal.connect(self.handle_signal)

        self._widget.graphicsView_top.setScene(self._qGraphicsSceneTop)
        self._widget.graphicsView_bottom.setScene(self._qGraphicsSceneBottom)
        self._widget.graphicsView_top_debug.setScene(self._qGraphicsSceneTopDebug)
        self._widget.graphicsView_bottom_debug.setScene(self._qGraphicsSceneBottomDebug)

        self._qGraphicsSceneTop.addItem(self._qPixmapItemTop)
        self._qGraphicsSceneBottom.addItem(self._qPixmapItemBottom)
        self._qGraphicsSceneTopDebug.addItem(self._qPixmapItemTopDebug)
        self._qGraphicsSceneBottomDebug.addItem(self._qPixmapItemBottomDebug)

        self._qPixmapItemBottom.mousePressEvent = self.on_qPixmapItemBottom_clicked

        self._widget.resizeEvent = self.on_resize

        self._widget.pushButton_reset.clicked.connect(self.on_pushButton_reset_clicked)
        self._widget.pushButton_send.clicked.connect(self.on_pushButton_send_clicked)

        self._widget.pushButton_reset.setEnabled(False)
        self._widget.pushButton_send.setEnabled(False)

        # Point publisher
        self._pub_points = rospy.Publisher("robostylus_planner/points", Point2D, queue_size=10)

        # Console screen images subscribers
        self._sub_top = rospy.Subscriber("/robostylus_camera/top_screen", Image, self.image_callback, callback_args=[self._qPixmapItemTop, self._widget.graphicsView_top])
        self._sub_bottom = rospy.Subscriber("/robostylus_camera/bottom_screen", Image, self.image_callback, callback_args=[self._qPixmapItemBottom, self._widget.graphicsView_bottom])

        # Debug image subscribers
        self._sub_top_debug = rospy.Subscriber("/robostylus_gui/top_debug", Image, self.image_callback, callback_args=[self._qPixmapItemTopDebug, self._widget.graphicsView_top_debug])
        self._sub_bottom_debug = rospy.Subscriber("/robostylus_gui/bottom_debug", Image, self.image_callback, callback_args=[self._qPixmapItemBottomDebug, self._widget.graphicsView_bottom_debug])

        self.on_checkBox_debug_changed(self._widget.checkBox_debug.checkState())

    # Resize images in viewports
    def on_resize(self, event):
        self._widget.graphicsView_top.fitInView(self._qPixmapItemTop, Qt.KeepAspectRatio)
        self._widget.graphicsView_bottom.fitInView(self._qPixmapItemBottom, Qt.KeepAspectRatio)
        self._widget.graphicsView_top_debug.fitInView(self._qPixmapItemTopDebug, Qt.KeepAspectRatio)
        self._widget.graphicsView_bottom_debug.fitInView(self._qPixmapItemBottomDebug, Qt.KeepAspectRatio)

    # Remove markers in bottom screen
    def on_pushButton_reset_clicked(self):
        for qEllipse in self._qEllipseObjects:
            self._qGraphicsSceneBottom.removeItem(qEllipse)

        self._qEllipseObjects = []

        self._widget.pushButton_reset.setEnabled(False)
        self._widget.pushButton_send.setEnabled(False)

    # Remove markers in bottom screen and send to the robot
    def on_pushButton_send_clicked(self):
        for qEllipse in self._qEllipseObjects:
            qPoint = qEllipse.rect().center()
            self._pub_points.publish(Point2D(qPoint.x(), qPoint.y()))

        self.on_pushButton_reset_clicked()

    # ROS callback for images
    def image_callback(self, image, args):
        pixmap, view = args
        self._signal.emit(image, pixmap, view)

    # Show received image in its viewport
    def handle_signal(self, image, qPixmapItem, qGraphicsView):
        qImage = QImage(image.data, image.width, image.height, image.step, QImage.Format_RGB888)
        qPixmap = QPixmap()
        qPixmap.convertFromImage(qImage)

        qPixmapItem.setPixmap(qPixmap)
        qGraphicsView.fitInView(qPixmapItem, Qt.KeepAspectRatio)

    # Paint marker on bottom screen
    def on_qPixmapItemBottom_clicked(self, ev):
        radius = 10
        pen = QPen(Qt.red, Qt.SolidPattern)
        pen.setWidth(5)
        qEllipse = self._qGraphicsSceneBottom.addEllipse(ev.pos().x() - radius, ev.pos().y() - radius, 2 * radius, 2 * radius, pen, QBrush())
        self._qEllipseObjects.append(qEllipse)

        self._widget.pushButton_reset.setEnabled(True)
        self._widget.pushButton_send.setEnabled(True)

    # Show or hide debug images
    def on_checkBox_debug_changed(self, state):
        if state == Qt.Checked:
            self._widget.groupBox_debug.show()
        elif state == Qt.Unchecked:
            self._widget.groupBox_debug.hide()

        self.on_resize(None);

    def shutdown_plugin(self):
        self._sub_top.unregister()
        self._sub_bottom.unregister()
        self._pub_points.unregister()

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value("checkBox_debug_state", self._widget.checkBox_debug.checkState())

    def restore_settings(self, plugin_settings, instance_settings):
        self._widget.checkBox_debug.setCheckState(int(instance_settings.value("checkBox_debug_state") or Qt.Unchecked))
