import os
import sys
from PyQt5.uic import loadUi
from PyQt5.QtWidgets import QDialog, QApplication, QStackedWidget, QWidget
from commandexec import CommandExecutor, NoConnectionError, commandexec_stop, execute_command, COMMAND_SIT, COMMAND_STAND
from gestrec import Gestrec, gestrec_stop, gestrec_on, gestrec_off

ui_startwindow = 'ui/start_window.ui'
ui_control_window = 'ui/control_window.ui'

commandexec = CommandExecutor()
gesterc = Gestrec()


class StartScreen(QDialog):
    def __init__(self):
        super(StartScreen, self).__init__()
        loadUi(ui_startwindow, self)
        self.connectButton.clicked.connect(self.go_to_control)
        self.exitButton.clicked.connect(self.close_app)

    def go_to_control(self):
        try:
            #commandexec.start()
            control = ControlScreen()
            widget.addWidget(control)
            widget.setCurrentWidget(control)
        except NoConnectionError as ex:
            self.errorLabel.setText(ex.message)

    def close_app(self):
        commandexec_stop()
        sys.exit()


class ControlScreen(QDialog):
    def __init__(self):
        super(ControlScreen, self).__init__()
        loadUi(ui_control_window, self)
        self.activateButton.clicked.connect(self.start_recognition)
        self.stopButton.clicked.connect(self.stop_recognition)
        self.exitButton.clicked.connect(self.close_app)
        self.sitButton.clicked.connect(self.sit_down)
        self.standButton.clicked.connect(self.stand_up)
        gesterc.start()

    def start_recognition(self):
        gestrec_on()
        self.recognitionLabel.setText("Gesture Recognition: on ")

    def stop_recognition(self):
        gestrec_off()
        self.recognitionLabel.setText("Gesture Recognition: off")

    def sit_down(self):
        execute_command(COMMAND_SIT)

    def stand_up(self):
        execute_command(COMMAND_STAND)

    def close_app(self):
        gestrec_stop()
        commandexec_stop()
        sys.exit()


if __name__ == '__main__':
    os.environ["QT_AUTO_SCREEN_SCALE_FACTOR"] = "1"
    app = QApplication(sys.argv)
    start = StartScreen()
    # start = ControlScreen()
    widget = QStackedWidget()
    widget.addWidget(start)
    widget.setFixedHeight(604)
    widget.setFixedWidth(701)
    widget.show()

    try:
        sys.exit(app.exec())
    except:
        print("Exiting")
