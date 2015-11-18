import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget,QIcon,QGraphicsScene,QImage,QGraphicsPixmapItem,QPixmap
from python_qt_binding.QtCore import Qt,qWarning,QRectF,QSize

import rosservice
from sensor_msgs.msg import Image

CALIBRATION_SERVICE_TYPE_NAME = "CalibrateCamera"

class RQT_Calibrate(Plugin):

    _services = {}
    _selected_service = {}
    _qGraphicsScene = QGraphicsScene()

    def __init__(self, context):
        super(RQT_Calibrate, self).__init__(context)

        # Give QObjects reasonable names
        self.setObjectName('RQT_Calibrate')

        # Create QWidget
        self._widget = QWidget()

        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('robostylus_gui'), 'resource', 'rqt_calibrate.ui')

        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)

        # Give QObjects reasonable names
        self._widget.setObjectName('RQT_Calibrate_UI')
        self._widget.setWindowTitle('Calibrate')

        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget).
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)

        # Configure individual widgets
        self._widget.pushButton_refresh.setIcon(QIcon.fromTheme('view-refresh'))
        self._widget.pushButton_refresh.clicked.connect(self.on_pushButton_refresh_clicked)

        self._widget.comboBox_services.currentIndexChanged['QString'].connect(self.on_comboBox_services_currentIndexChanged)

        self._widget.pushButton_calibrate.clicked.connect(self.on_pushButton_calibrate_clicked)

        self._widget.graphicsView.setScene(self._qGraphicsScene)
        self._widget.graphicsView.wheelEvent = self.on_graphicsView_wheelEvent

        self.on_pushButton_refresh_clicked()

    # Refresh service list
    def on_pushButton_refresh_clicked(self):
        self._services = {}

        # Select calibration services
        service_names = rosservice.get_service_list()
        for service_name in service_names:
            try:
                current_service = rosservice.get_service_class_by_name(service_name)
                if current_service.__name__ == CALIBRATION_SERVICE_TYPE_NAME:
                    self._services[service_name] = current_service
            except (rosservice.ROSServiceException, rosservice.ROSServiceIOException) as e:
                qWarning('on_pushButton_refresh_clicked(): could not get class of service %s:\n%s' % (service_name, e))

        self._widget.comboBox_services.clear()
        self._widget.comboBox_services.addItems(sorted(self._services.keys()))

    # Update selected service
    def on_comboBox_services_currentIndexChanged(self, service_name):
        service_name = str(service_name)
        if not service_name:
            return

        # Get service info and calling helper
        self._selected_service = {}
        self._selected_service['service_name'] = service_name
        self._selected_service['service_class'] = self._services[service_name]
        self._selected_service['service_proxy'] = rospy.ServiceProxy(service_name, self._selected_service['service_class'])

        self._widget.pushButton_calibrate.setEnabled(True)

    # Perform calibration service call
    def on_pushButton_calibrate_clicked(self):

        self._widget.pushButton_calibrate.setEnabled(False)

        status_icon = QIcon.fromTheme('stock_no')

        # Fill request information
        request = self._selected_service['service_class']._request_class()
        request.target_hue.data = self._widget.spinBox_hue.value()
        request.mult_diffs.data = self._widget.spinBox_diff.value()
        request.mult_sat.data = self._widget.spinBox_sat.value()
        try:
            # Call service
            response = self._selected_service['service_proxy'](request)
        except rospy.ServiceException as e:
            qWarning('on_pushButton_calibrate_clicked(): error calling service "%s":\n%s' % (self._selected_service['service_name'], e))
        else:
            # Get debug image and show
            img = response.image_debug
            qImage = QImage(img.data, img.width, img.height, img.step, QImage.Format_RGB888)
            qPixmap = QPixmap()
            qPixmap.convertFromImage(qImage)
            pixmapItem = QGraphicsPixmapItem(qPixmap)
            self._qGraphicsScene.clear()
            self._qGraphicsScene.addItem(pixmapItem)

            self._widget.graphicsView.fitInView(QRectF(qPixmap.rect()), Qt.KeepAspectRatio)

            if response.success.data == True:
                status_icon = QIcon.fromTheme('stock_yes')

        self._widget.label_status.setPixmap(status_icon.pixmap(status_icon.actualSize(QSize(24, 24))))

        self._widget.pushButton_calibrate.setEnabled(True)

    # Handle zoom
    def on_graphicsView_wheelEvent(self, event):
        scaleFactor = 1.1
        if event.delta() > 0:
            self._widget.graphicsView.scale(scaleFactor, scaleFactor)
        else:
            self._widget.graphicsView.scale(1.0 / scaleFactor, 1.0 / scaleFactor)

    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('spinbox_hue_value', self._widget.spinBox_hue.value())
        instance_settings.set_value('spinbox_diff_value', self._widget.spinBox_diff.value())
        instance_settings.set_value('spinbox_sat_value', self._widget.spinBox_sat.value())

    def restore_settings(self, plugin_settings, instance_settings):
        self._widget.spinBox_hue.setValue(int(instance_settings.value('spinbox_hue_value') or 0))
        self._widget.spinBox_diff.setValue(float(instance_settings.value('spinbox_diff_value') or 0))
        self._widget.spinBox_sat.setValue(float(instance_settings.value('spinbox_sat_value') or 0))
