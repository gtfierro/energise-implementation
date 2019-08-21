
import numpy as np
from scipy.linalg import solve_discrete_are as dare
from numpy.linalg import inv

#using matrices rather than nd.arrays bc ecah controller only needs 2 dimensions, makes math easier
#all voltages, powers and impedances in pu

class LQRcontroller:
    def __init__(self,nphases,timesteplength,Qcost,Rcost,VmagRef,VangRef,controllerUpdateCadence,saturated,lpAlpha,linearizeplant,lam,ninit,ZskAsMatrix,Zskinit,Gt):
        self.nphases = nphases
        self.VmagRef = VmagRef
        self.VangRef = VangRef
        self.V0 = np.hstack((self.VmagRef,self.VangRef))
        self.VmagTarg = np.NaN
        self.VangTarg = np.NaN
        self.timesteplength = timesteplength

        #Controller Parameters
        self.Qcost = Qcost
        self.Rcost = Rcost
        self.controllerUpdateCadence = controllerUpdateCadence
        self.saturated = saturated
        #DOBC parameters
        self.lpAlpha = lpAlpha # low pass filter alpha, larger changes disturbance faster (more noise sensitive)
        self.linearizeplant = linearizeplant
        #REIE parameters
        self.lam = lam # 0 < lam < 1, smaller lam cahnges state faster (more noise sensitive)
        self.ninit = ninit # window to use if initializedwData = 1 (not used)
        self.ZskAsMatrix = ZskAsMatrix # need ZskAsMatrix to account for coupling of ground impedance (p 78 of Kersting)

        self.Zskest = Zskinit
        self.Zskestinit = Zskinit
        self.Gt = Gt
        (self.A, self.B) = self.makeABmatrices(Zskinit,timesteplength)
        self.K = np.asmatrix(np.zeros((2*nphases,4*nphases)))

        self.state = np.asmatrix(np.zeros(4*nphases))
        self.u = np.asmatrix(np.zeros(2*nphases))
        self.d = np.asmatrix(np.zeros(2*nphases))
        self.IcompPrev = np.NaN
        self.VcompPrev = np.NaN


####################################################

    def setParameters(self):
        return

    def setVref(self,VmagRef,VangRef):
        return

    def setVtarget(self,VmagTarg,VangTarg):
        self.VmagTarg = VmagTarg
        self.VangTarg = VangTarg
        return

    def setLQRweights(self):
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
        return


    def updateController(self):
        S = np.asmatrix(dare(self.A,self.B,self.Qcost,self.Rcost))
        self.K = -(self.Rcost+self.B.T*S*self.B).I*self.B.T*S*self.A; # neg for neg feedback
        # K = cl.lqr(A,B,Qcost,Rcost) #continuous only, I think
        return self.K


    def calcGtAndZskinit(self,Zsktru):
        Gt = np.asmatrix(np.eye(self.nphases)) # large Gts can make the system go unstable
        Gt = np.asmatrix(np.eye(self.nphases))*.01
        ZskestInit = np.NAN
        #ZskestInit = Zsktru*(1 + randn*.1) #arbitrary noise in the Zeffestimate
        return(Gt,ZskestInit)


    def pfEqns3phase(self,VmagTarg,VangTarg,Zskest):
        error('not built')
        return


    def APCupdate(self,Vmag,Vang,Icomp,ts):
        #Internal Controller Accounting
        #set statek to Vmag and Vang and increment integrator
        #need to adjust this section if you want non-controlled nodes
        #for j in np.arange(3):
        Vcomp = Vmag*np.cos(Vang) + Vmag*np.sin(Vang)*1j #check
        #Icomp = Imag*np.cos(Iang) + Imag*np.sin(Iang)*1j
        Vmag = np.asmatrix(Vmag)
        Vang = np.asmatrix(Vang)
        #Imag = np.asmatrix(Imag)
        #Iang = np.asmatrix(Iang)
        Vcomp = np.asmatrix(Vcomp)
        Icomp = np.asmatrix(Icomp)

        if self.saturated: #saturated signal has to come from loads, it can't be measured locally if there are unmeasured loads downstream from lpbc node
            self.state = np.hstack((Vmag-self.VmagTarg,Vang-self.VangTarg,self.state[0,self.nphases*2:self.nphases*4]))
        else:
            self.state = np.hstack((Vmag-self.VmagTarg,Vang-self.VangTarg,self.timesteplength*self.state[0,0:self.nphases*2]+self.state[0,self.nphases*2:self.nphases*4]))

        R = np.real(self.Zskest)
        X = np.imag(self.Zskest)
        Babbrev = np.vstack((np.hstack((-R, -X)),np.hstack((-X, R))))

        #Disturbance Observer
        #disturbance is calculated using the u and d from the previous timestep, but the V from the current timestep
        #LPF: exponentially weighted moving average filter
        if self.linearizeplant:
            ueff = np.linalg.pinv(Babbrev)*(np.hstack((Vmag,Vang))-self.V0).T #is this feedback linearization?
        else:
            ueff = self.pfEqns3phase(Vmag,Vang,Zskest) #havent built these yet #ueff through Zeff would give Vmeas
        if ts != 0: #better way to do this wo passing in ts?
            dm = ueff.T - self.u #dm for d measurement
            self.d = (1-self.lpAlpha)*self.d + self.lpAlpha*dm  #this is the moving average filter for estimating the disturbance
        else:
            dm = ueff;
            self.d = dm;

        #update uref
        if self.linearizeplant: #uses DOBC to make the plant (PF equations) look like a linear model
            uref = (np.linalg.pinv(Babbrev)*(np.hstack((self.VmagTarg,self.VangTarg))-self.V0).T).T;
        else: # DOBC does not make the PF equations look like a linear model
            uref = self.pfEqns3phase(VmagTarg,VangTarg,Zskest) #havent built these yet

        #Feedback Control input for next round
        #uref is the naive power injection that gives Vref, added to
        #offset the Vref that is subtracted to make the state. This allows
        #the 4 state LQR to be used, rather than an LQTC.
        #ueff is the power vector that would give V (measured) through
        #Zeff, used in DOBC to determine the disturbance.
        self.u = (self.K*self.state.T).T + uref - self.d #d adjustment here, this is DOBC

        #Estimate Zeff (direct formulation)
        # w (the parameter that is estimated) in Haykin is Y^H, not Y
        # http://users.ics.forth.gr/tsakalid/UVEG09/Book/Haykin-AFT(3rd.Ed.)_Chapter13.pdf
        #get difference in time measurements
        if ts != 0: # There arent any previous measurements at t=1, so you cant update Zeff
            dtVt = (Vcomp - self.VcompPrev).T #these are vertical vectors
            dtIt = (Icomp - self.IcompPrev).T
            #Recursive Least Squares estimate of Zeff
            if self.ZskAsMatrix:
                self.Gt = self.Gt/self.lam - (self.Gt*(dtIt*dtIt.H)*self.Gt)/(self.lam**2*(1 + dtIt.H*self.Gt*dtIt/self.lam))
                err = dtVt - self.Zskest*dtIt
                self.Zskest = np.asmatrix(self.Zskest.H + self.Gt*dtIt*err.H).H #if err is negative and Gt is too big, Zeffest will go negative
                #self.Zskest = self.Zskest + err*dtIt.H*self.Gt.H
                self.Zskest = (self.Zskest + self.Zskest.T)/2;
            else:
                for p in np.arange(self.nphases):
                    self.Gt[p,p] = self.Gt[p,p]/self.lam - (self.Gt[p,p]*(dtIt[p]*dtIt[p].H)*self.Gt[p,p])/(self.lam**2*(1 + dtIt[p].H*self.Gt[p,p]*dtIt[p]/self.lam))
                    err = dtVt[p] - self.Zskest[p,p]*dtIt[p]
                    self.Zskest[p,p] = (self.Zskest[p,p].H + self.Gt[p,p]*dtIt[p]*err.H).H #if err is negative and Gt is too big, Zeffest will go negative
        for p in np.arange(self.nphases): #not sure this is the right way to do this yet
            if np.real(self.Zskest[p,p]) < 0: #|| np.imag(self.Zskest[p,p]) < 0: #these bounds can be adjusted to serve as bumper rails using prior information
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
