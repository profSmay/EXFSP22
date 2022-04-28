from Car_GUI import Ui_Form
import sys
from PyQt5 import QtCore as qtc
from PyQt5 import QtWidgets as qtw
from PyQt5 import QtGui as qtg
from QuarterCarModel_stem import CarController

#these imports are necessary for drawing a matplot lib graph on my GUI
#no simple widget for this exists in QT Designer, so I have to add the widget in code.
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure

class MainWindow(qtw.QWidget, Ui_Form):
    def __init__(self):
        """
        Main window constructor.
        """
        super().__init__()

        self.setupUi(self)

        # creating a canvas to draw a figure for the car model
        self.figure=Figure(tight_layout=True, frameon=True, facecolor='none')
        self.canvas = FigureCanvasQTAgg(self.figure)

        self.layout_MainHorizontal.addWidget(self.canvas)

        #setup car controller
        self.controller=CarController(self.figure.add_subplot())  # instantiate controller with axes argument
        w=[self.le_mcar, self.le_CarSpeed, self.le_k1, self.le_c1, self.le_m2, self.le_k2, self.le_AngDeg, self.le_tMax]
        w+=[self.chk_LogX, self.chk_LogY, self.chk_LogAccel, self.chk_PlotAccel, self.chk_IncludeAccel, self.lbl_MaxMinInfo]
        self.controller.setWidgets(w)

        # connect clicked signal of calculate button
        self.btn_calculate.clicked.connect(self.doCalculate)
        self.pb_Optimize.clicked.connect(self.doOptimize)
        self.chk_LogX.stateChanged.connect(self.doPlot)
        self.chk_LogY.stateChanged.connect(self.doPlot)
        self.chk_LogAccel.stateChanged.connect(self.doPlot)
        self.chk_PlotAccel.stateChanged.connect(self.doPlot)
        self.show()

    def doCalculate(self):
        self.controller.set()
        self.canvas.draw()

    def doOptimize(self):
        self.controller.OptimizeSuspension()
        self.canvas.draw()

    def doPlot(self):
        self.controller.doPlot()
        self.canvas.draw()

if __name__ == '__main__':
    app = qtw.QApplication(sys.argv)
    mw = MainWindow()
    mw.setWindowTitle('Quarter Car Model')
    sys.exit(app.exec())
