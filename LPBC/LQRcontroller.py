
import numpy as np
from scipy.linalg import solve_discrete_are as dare
from numpy.linalg import inv

#using matrices rather than nd.arrays bc ecah controller only needs 2 dimensions, makes math easier
#all voltages, powers and impedances in pu

class LQRcontroller:
    def __init__(self,nphases,timesteplength,Qcost,Rcost,VmagRef,VangRef,Zskinit,use_Zsk_est,currentMeasExists=1,Gt=np.asmatrix(np.eye(self.nphases))*.01,lpAlpha=.1,lam=.99,controllerUpdateCadence=1,linearizeplant=1):
        self.nphases = nphases
        self.VmagRef = VmagRef # = 0 if Vmag is given as relative already
        self.VangRef = VangRef
        self.V0 = np.hstack((self.VmagRef,self.VangRef))
        self.VmagTarg = np.NaN
        self.VangTarg = np.NaN
        self.timesteplength = timesteplength
        self.currentMeasExists = currentMeasExists

        #Controller Parameters
        self.Qcost = Qcost
        self.Rcost = Rcost
        self.controllerUpdateCadence = controllerUpdateCadence
        self.saturated = 0

        self.lpAlpha = lpAlpha # low pass filter alpha, larger changes disturbance faster (more noise sensitive)
        self.linearizeplant = linearizeplant

        self.lam = lam # 0 < lam < 1, smaller lam cahnges state faster (more noise sensitive)

        self.use_Zsk_est = use_Zsk_est
        self.Zskest = Zskinit
        self.Gt = Gt
        (self.A, self.B) = self.makeABmatrices(Zskinit,timesteplength)
        self.K = np.asmatrix(np.zeros((2*nphases,4*nphases)))

        self.state = np.asmatrix(np.zeros(4*nphases))
        self.u = np.asmatrix(np.zeros(2*nphases))
        self.d = np.asmatrix(np.zeros(2*nphases))
        self.IcompPrev = np.NaN
        self.VcompPrev = np.NaN


####################################################

    def setVtarget(self,VmagTarg,VangTarg):
        self.VmagTarg = VmagTarg
        self.VangTarg = VangTarg
        return

    def makeABmatrices(self,Zsk,timesteplength):
        A = np.vstack((np.zeros((2*self.nphases,4*self.nphases)),np.hstack((timesteplength*np.eye(2*self.nphases),np.eye(2*self.nphases)))))
        R = np.real(Zsk)
        X = np.imag(Zsk)
        B = np.vstack((np.hstack((-R, -X)),np.hstack((-X, R)),np.zeros((2*self.nphases,2*self.nphases))))
        self.A = np.asmatrix(A)
        self.B = np.asmatrix(B)
        return (self.A,self.B)

    def updateZskAndB(self):
        error('not built')
        return

    def updateController(self):
        S = np.asmatrix(dare(self.A,self.B,self.Qcost,self.Rcost))
        self.K = -(self.Rcost+self.B.T*S*self.B).I*self.B.T*S*self.A; # neg for neg feedback
        # K = cl.lqr(A,B,Qcost,Rcost) #continuous only, I think
        return self.K


    def calcGtAndZskinit(self,Zsk):
        #TODO
        Gt = np.asmatrix(np.eye(self.nphases))*.01
        return(Gt)


    def pfEqns3phase(self,VmagTarg,VangTarg,Zskest):
        error('not built')
        return


    def LQRupdate(self,Vmag,Vang,Icomp,iteration_counter): #HHERE shape of Vmag after asmatrix
        #Internal Controller Accounting
        #set statek to Vmag and Vang and increment integrator
        Vcomp = Vmag*np.cos(Vang) + Vmag*np.sin(Vang)*1j
        Vmag = np.asmatrix(Vmag)
        Vang = np.asmatrix(Vang)
        Vcomp = np.asmatrix(Vcomp)
        Icomp = np.asmatrix(Icomp)

        if self.saturated:
            self.state = np.hstack((Vmag-self.VmagTarg,Vang-self.VangTarg,self.state[0,self.nphases*2:self.nphases*4]))
        else:
            self.state = np.hstack((Vmag-self.VmagTarg,Vang-self.VangTarg,self.timesteplength*self.state[0,0:self.nphases*2]+self.state[0,self.nphases*2:self.nphases*4]))

        R = np.real(self.Zskest)
        X = np.imag(self.Zskest)
        Babbrev = np.vstack((np.hstack((-R, -X)),np.hstack((-X, R))))
        if np.mod(iteration_counter-1,self.controllerUpdateCadence) == 0:
            self.K = self.updateController()

        #DOBC
        if self.linearizeplant:
            ueff = np.linalg.pinv(Babbrev)*(np.hstack((Vmag,Vang))-self.V0).T
        else:
            ueff = self.pfEqns3phase(Vmag,Vang,Zskest) #havent built these yet #ueff through Zeff would give Vmeas
        if iteration_counter != 1: #iteration_counter is 1 in the first call
            dm = ueff.T - self.u #dm for d measurement
            self.d = (1-self.lpAlpha)*self.d + self.lpAlpha*dm
        else:
            dm = ueff;
            self.d = dm;

        #update uref
        if self.linearizeplant:
            uref = (np.linalg.pinv(Babbrev)*(np.hstack((self.VmagTarg,self.VangTarg))-self.V0).T).T;
        else:
            uref = self.pfEqns3phase(VmagTarg,VangTarg,Zskest) #havent built these yet

        #Feedback Control input for next round
        self.u = (self.K*self.state.T).T + uref - self.d

        #Estimate Zeff
        if self.use_Zsk_est == 1 and (currentMeasExists == 1 or saturated == 0): #only run Zsk est if you have a current measurement or the actuators arent saturated
            if iteration_counter != 1: # There arent any previous measurements at t=1, so you cant update Zeff
                dtVt = (Vcomp - self.VcompPrev).T #these are vertical vectors
                dtIt = (Icomp - self.IcompPrev).T
                self.Gt = self.Gt/self.lam - (self.Gt*(dtIt*dtIt.H)*self.Gt)/(self.lam**2*(1 + dtIt.H*self.Gt*dtIt/self.lam))
                err = dtVt - self.Zskest*dtIt
                self.Zskest = np.asmatrix(self.Zskest.H + self.Gt*dtIt*err.H).H
                self.Zskest = (self.Zskest + self.Zskest.T)/2;
            for p in np.arange(self.nphases):
                if np.real(self.Zskest[p,p]) < 0:
                    self.Zskest = self.Zskestinit
                    self.Gt = self.Gt*.1
                    print('reset impedance estimator')

        #save measurements for next round
        self.IcompPrev = Icomp;
        self.VcompPrev = Vcomp;

        #LQR defines u as positive for power flowing out of the network (due to the signs of the PF linearization)
        #but general LPBC convention is power is defined as postive into the network
        #returns powers in pu, I believe
        Plpbc = np.asarray(-self.u[0,0:self.nphases])
        Qlpbc = np.asarray(-self.u[0,self.nphases:2*self.nphases])
        return (Plpbc,Qlpbc)
