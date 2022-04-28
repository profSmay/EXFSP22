from scipy.integrate import odeint
from scipy.optimize import minimize
import matplotlib.pyplot as plt
import numpy as np
import math
from PyQt5 import QtWidgets as qtw

class CarModel():
    """
    I re-wrote the quarter car model as an object oriented program
    and used the MVC pattern.  This is the quarter car model.  It just
    stores information about the car and results of the ode calculation.
    """
    def __init__(self):
        """
        self.results to hold results of odeint solution
        self.t time vector for odeint and for plotting
        self.tramp is time required to climb the ramp
        self.angrad is the ramp angle in radians
        self.ymag is the ramp height in m
        """
        self.results = []
        self.tmax = 3.0  # limit of timespan for simulation in seconds
        self.t = np.linspace(0, self.tmax, 200)
        self.tramp = 1.0  # time to traverse the ramp in seconds
        self.angrad = 0.1
        self.ymag = 6.0 / (12 * 3.3)  # ramp height in meters.  default is 0.1515 m
        self.yangdeg = 45.0  # ramp angle in degrees.  default is 45
        self.results = None
        self.m1 = 450  # mass of car body in kg
        self.m2 = 20  # mass of wheel in kg
        self.c1 = 4500  # damping coefficient in N*s/m
        self.k1 = 15000  # spring constant of suspension in N/m
        self.k2 = 90000  # spring constant of tire in N/m
        self.v = 120.0  # velocity of car in kph
        #self.mink1=(self.m1*9.81)/(16.0*25.4/1000.0)
        self.mink1=(self.m1*9.81)/(6.0*25.4/1000.0)
        self.maxk1=(self.m1*9.81)/(3.0*25.4/1000.0)
        self.mink2=((self.m1+self.m2)*9.81)/(1.5*25.4/1000.0)
        self.maxk2=((self.m1+self.m2)*9.81)/(0.75*25.4/1000.0)
        self.accel=None
        self.accelMax=0
        self.accelLim=1.5
        self.SSE=0.0

class CarController():
    def __init__(self, ax=None):
        """
        This is the controller I am using for the quarter car model.
        """
        self.model = CarModel()
        self.view = CarView()

        self.chk_IncludeAccel=qtw.QCheckBox()

        self.view.ax = ax  # axes for the plotting using view
        if ax is not None:
            self.view.ax1=ax.twinx()

    def ode_system(self, X, t):
        # define the forcing function equation for the linear ramp
        # It takes self.tramp time to climb the ramp, so y position is
        # a linear function of time.
        if t < self.model.tramp:
            y = self.model.ymag * (t / self.model.tramp)
        else:
            y = self.model.ymag

        x1 = X[0]  # car position in vertical direction
        x1dot = X[1]  # car velocity  in vertical direction
        x2 = X[2]  # wheel position in vertical direction
        x2dot = X[3]  # wheel velocity in vertical direction

        # write the non-trivial equations in vertical direction
        x1ddot = (1 / self.model.m1) * (self.model.c1 * (x2dot - x1dot) + self.model.k1 * (x2 - x1))
        x2ddot = (1 / self.model.m2) * (
                    -self.model.c1 * (x2dot - x1dot) - self.model.k1 * (x2 - x1) + self.model.k2 * (y - x2))

        # return the derivatives of the input state vector
        return [x1dot, x1ddot, x2dot, x2ddot]

    def set(self, doCalc=True):
        """
        I will first set the basic properties of the car model and then calculate the result
        in another function doCalc.
        """
        #Step 1.  Read from the widgets
        self.model.m1 = float(self.view.le_m1.text())
        self.model.m2 = float(self.view.le_m2.text())
        self.model.c1 = float(self.view.le_c1.text())
        self.model.k1 = float(self.view.le_k1.text())
        self.model.k2 = float(self.view.le_k2.text())
        self.model.v = float(self.view.le_v.text())

        #recalculate min and max k values
        self.model.mink1=(self.model.m1*9.81)/(6.0*25.4/1000.0)
        self.model.maxk1=(self.model.m1*9.81)/(3.0*25.4/1000.0)
        self.model.mink2=((self.model.m1+self.model.m2)*9.81)/(1.5*25.4/1000.0)
        self.model.maxk2=((self.model.m1+self.model.m2)*9.81)/(0.75*25.4/1000.0)

        ymag=6.0/(12.0*3.3)   #This is the height of the ramp in m
        if ymag is not None:
            self.model.ymag = ymag
        self.model.yangdeg = float(self.view.le_ang.text())
        self.model.tmax = float(self.view.le_tmax.text())
        if(doCalc):
            self.doCalc()
        self.SSE((self.model.k1, self.model.c1, self.model.k2), optimizing=False)
        self.view.updateView(self.model)

    def setWidgets(self, w):
        self.view.setWidgets(w)
        self.chk_IncludeAccel=self.view.chk_IncludeAccel

    def doCalc(self, doPlot=True, doAccel=True):
        """
        This solves the differential equations for the quarter car model.
        :param doPlot:
        :param doAccel:
        :return:
        """
        v = 1000 * self.model.v / 3600  # convert speed to m/s from kph
        self.model.angrad = self.model.yangdeg * math.pi / 180.0  # convert angle to radians
        self.model.tramp = self.model.ymag / (math.sin(self.model.angrad) * v)  # calculate time to traverse ramp

        self.model.t=np.linspace(0,self.model.tmax,2000)
        ic = [0, 0, 0, 0]
        # run odeint solver
        self.model.results = odeint(self.ode_system, ic, self.model.t)
        if doAccel:
            self.calcAccel()
        if doPlot:
            self.doPlot()

    def calcAccel(self):
        """
        Calculate the acceleration in the vertical direction using the forward difference formula.
        """
        N=len(self.model.t)
        self.model.accel=np.zeros(shape=N)
        vel=self.model.results[:,1]
        for i in range(N):
            if i==N-1:
                h = self.model.t[i] - self.model.t[i-1]
                self.model.accel[i]=(vel[i]-vel[i-1])/(9.81*h)  # backward difference of velocity
            else:
                h = self.model.t[i + 1] - self.model.t[i]
                self.model.accel[i] = (vel[i + 1] - vel[i]) / (9.81 * h)  # forward difference of velocity
            # else:
            #     self.model.accel[i]=(vel[i+1]-vel[i-1])/(9.81*2.0*h)  # central difference of velocity
        self.model.accelMax=self.model.accel.max()
        return True

    def OptimizeSuspension(self):
        """
        Step 1:  set parameters based on GUI inputs by calling self.set(doCalc=False)
        Step 2:  make an initial guess for k1, c1, k2
        Step 3:  optimize the suspension
        :return:
        """
        #Step 1:
        #$JES MISSING CODE HERE$

        #Step 2:
        #JES MISSING CODE HERE$

        #Step 3:
        #JES MISSING CODE HERE$

        self.view.updateView(self.model)

    def SSE(self, vals, optimizing=True):
        """
        Calculates the sum of square errors between the contour of the road and the car body.
        :param vals:
        :param optimizing:
        :return:
        """
        k1, c1, k2=vals  #unpack the new values for k1, c1, k2
        self.model.k1=k1
        self.model.c1=c1
        self.model.k2=k2
        self.doCalc(doPlot=False)  #solve the odesystem with the new values of k1, c1, k2
        SSE=0
        for i in range(len(self.model.results[:,0])):
            t=self.model.t[i]
            y=self.model.results[:,0][i]
            if t < self.model.tramp:
                ytarget = self.model.ymag * (t / self.model.tramp)
            else:
                ytarget = self.model.ymag
            SSE+=(y-ytarget)**2

        #some penalty functions if the constants are too small
        if optimizing:
            if k1<self.model.mink1 or k1>self.model.maxk1:
                SSE+=1000000
            if c1<10:
                SSE+=1000000
            if k2<self.model.mink2 or k2>self.model.maxk2:
                SSE+=1000000

            # I'm overlaying a gradient in the acceleration limit that scales with distance from a target squared.
            if self.model.accelMax>self.model.accelLim and self.chk_IncludeAccel.isChecked():
                # need to soften suspension
                SSE+=(self.model.accelMax-self.model.accelLim)**2
        self.model.SSE=SSE
        return SSE

    def doPlot(self):
        self.view.doPlot(self.model)

class CarView():
    def __init__(self):
        self.ax = None
        self.ax1=None
        self.le_k1=qtw.QLineEdit()
        self.le_c1=qtw.QLineEdit()
        self.le_k2=qtw.QLineEdit()
        self.le_m1=qtw.QLineEdit()
        self.le_m2=qtw.QLineEdit()
        self.le_v=qtw.QLineEdit()
        self.le_ang=qtw.QLineEdit()
        self.le_tmax=qtw.QLineEdit()
        self.chk_IncludeAccel=qtw.QCheckBox()
        self.chk_ShowAccel=qtw.QCheckBox()
        self.chk_LogX=qtw.QCheckBox()

    def setWidgets(self, w):
        self.le_m1=w[0]
        self.le_v=w[1]
        self.le_k1=w[2]
        self.le_c1=w[3]
        self.le_m2=w[4]
        self.le_k2=w[5]
        self.le_ang=w[6]
        self.le_tmax=w[7]
        self.chk_LogX=w[8]
        self.chk_LogY=w[9]
        self.chk_LogAccel=w[10]
        self.chk_ShowAccel=w[11]
        self.chk_IncludeAccel=w[12]
        self.lbl_MaxMinInfo=w[13]

    def updateView(self, model=None):
        self.le_m1.setText("{:0.2f}".format(model.m1))
        self.le_k1.setText("{:0.2f}".format(model.k1))
        self.le_c1.setText("{:0.2f}".format(model.c1))
        self.le_m2.setText("{:0.2f}".format(model.m2))
        self.le_k2.setText("{:0.2f}".format(model.k2))
        self.le_ang.setText("{:0.2f}".format(model.yangdeg))
        self.le_tmax.setText("{:0.2f}".format(model.tmax))
        stTmp="k1_min = {:0.2f}, k1_max = {:0.2f}\nk2_min = {:0.2f}, k2_max = {:0.2f}\n".format(model.mink1, model.maxk1, model.mink2, model.maxk2)
        stTmp+="SSE = {:0.2f}".format(model.SSE)
        self.lbl_MaxMinInfo.setText(stTmp)
        self.doPlot(model)

    def doPlot(self, model=None):
        if model.results is None:
            return
        ax = self.ax
        ax1=self.ax1
        # plot result of odeint solver
        QTPlotting = True  # assumes we are plotting onto a QT GUI form
        if ax == None:
            ax = plt.subplot()
            ax1=ax.twinx()
            QTPlotting = False  # actually, we are just using CLI and showing the plot
        ax.clear()
        ax1.clear()
        t=model.t
        ycar = model.results[:,0]
        ywheel=model.results[:,2]
        accel=model.accel

        if self.chk_LogX.isChecked():
            ax.set_xlim(0.001,model.tmax)
            ax.set_xscale('log')
        else:
            ax.set_xlim(0.0, model.tmax)
            ax.set_xscale('linear')

        if self.chk_LogY.isChecked():
            ax.set_ylim(0.0001,max(ycar.max(), ywheel.max()*1.05))
            ax.set_yscale('log')
        else:
            ax.set_ylim(0.0, max(ycar.max(), ywheel.max()*1.05))
            ax.set_yscale('linear')

        ax.plot(t, ycar, 'b-', label='Body Position')
        ax.plot(t, ywheel, 'r-', label='Wheel Position')
        if self.chk_ShowAccel.isChecked():
            ax1.plot(t, accel, 'g-', label='Body Accel')
            ax1.axhline(y=accel.max(), color='orange')  # horizontal line at accel.max()
            ax1.set_yscale('log' if self.chk_LogAccel.isChecked() else 'linear')

        # add axis labels
        ax.set_ylabel("Vertical Position (m)", fontsize='large' if QTPlotting else 'medium')
        ax.set_xlabel("time (s)", fontsize='large' if QTPlotting else 'medium')
        ax1.set_ylabel("Y'' (g)", fontsize = 'large' if QTPlotting else 'medium')
        ax.legend()

        ax.axvline(x=model.tramp)  # vertical line at tramp
        ax.axhline(y=model.ymag)  # horizontal line at ymag
        # modify the tick marks
        ax.tick_params(axis='both', which='both', direction='in', top=True,
                       labelsize='large' if QTPlotting else 'medium')  # format tick marks
        ax1.tick_params(axis='both', which='both', direction='in', right=True,
                       labelsize='large' if QTPlotting else 'medium')  # format tick marks
        # show the plot
        if QTPlotting == False:
            plt.show()

def main():
    QCM = CarController()
    QCM.doCalc()


if __name__ == '__main__':
    main()
