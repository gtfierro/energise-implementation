
import numpy as np
from scipy.linalg import solve_discrete_are as dare
from numpy.linalg import inv

#using matrices rather than nd.arrays bc ecah controller only needs 2 dimensions, makes math easier
#all voltages, powers and impedances in pu

#HHERE how to account for P and Q commands not being true

class LQRcontroller:
    def __init__(self,nphases,timesteplength,Qcost,Rcost,Zskinit,use_Zsk_est,currentMeasExists=1,lpAlpha=.1,lam=.99,Gt=None,controllerUpdateCadence=1,linearizeplant=1):
        self.nphases = nphases
        self.VmagRef = np.NaN
        self.VangRef = np.NaN
        self.V0 = np.hstack((self.VmagRef,self.VangRef))
        self.VmagTarg = np.NaN
        self.VangTarg = np.NaN
        self.timesteplength = timesteplength
        self.currentMeasExists = currentMeasExists
        self.iteration_counter = 0

        #Controller Parameters
        self.Qcost = np.asmatrix(Qcost)
        self.Rcost = np.asmatrix(Rcost)
        self.controllerUpdateCadence = controllerUpdateCadence

        self.lpAlpha = lpAlpha # low pass filter alpha, larger changes disturbance faster (more noise sensitive)
        self.linearizeplant = linearizeplant

        self.lam = lam # 0 < lam < 1, smaller lam changes Zskest faster

        self.use_Zsk_est = use_Zsk_est
        self.Zskestinit = Zskinit
        self.Zskest = Zskinit
        if Gt == None: #done bc its bad to initialize a variable to a mutable type https://opensource.com/article/17/6/3-things-i-did-wrong-learning-python
            self.Gt = np.asmatrix(np.eye(self.nphases))*.01
        else:
            self.Gt = Gt
        (self.A, self.B) = self.makeABmatrices(Zskinit,timesteplength)
        self.K = np.asmatrix(np.zeros((2*nphases,4*nphases)))

        self.state = np.asmatrix(np.zeros(4*nphases))
        self.u = np.asmatrix(np.zeros(2*nphases))
        self.d = np.asmatrix(np.zeros(2*nphases))
        self.IcompPrev = np.asmatrix(np.zeros(nphases,dtype=np.complex_)) #these will make row matrices
        self.VcompPrev = np.asmatrix(np.zeros(nphases,dtype=np.complex_))
        self.PcommandPrev = np.zeros(nphases) #want this as an np array
        self.QcommandPrev = np.zeros(nphases)


####################################################

    def setVref(self,VmagRef,VangRef):
        self.VmagRef = VmagRef
        self.VangRef = VangRef
        self.V0 = np.hstack((self.VmagRef,self.VangRef))
        return self.V0


    def setVtarget(self,VmagTarg,VangTarg):
        self.VmagTarg = VmagTarg
        self.VangTarg = VangTarg
        return


    def makeABmatrices(self,Zsk,timesteplength):
        A = np.vstack((np.zeros((2*self.nphases,4*self.nphases)),np.hstack((timesteplength*np.eye(2*self.nphases),np.eye(2*self.nphases)))))
        R = np.real(Zsk)
        X = np.imag(Zsk)
        B = np.vstack((np.hstack((-R, -X)),np.hstack((-X, R)),np.zeros((2*self.nphases,2*self.nphases))))
        A = np.asmatrix(A)
        B = np.asmatrix(B)
        return (A,B)


    def updateB(self, Zsk):
        R = np.real(self.Zskest)
        X = np.imag(self.Zskest)
        ze = np.asmatrix(np.zeros((2*self.nphases,2*self.nphases)))
        B = np.vstack((np.hstack((-R, -X)),np.hstack((-X, R)),np.asmatrix(np.zeros((2*self.nphases,2*self.nphases))))) #all the entries are numpy matrices so this makes a numpy matrix
        return B


    def updateController(self,A,B,Q,R):
        S = np.asmatrix(dare(A,B,Q,R)) #from scipy
        K = -(R+B.T*S*B).I*B.T*S*A # neg for neg feedback
        return K


    def calcGtAndZskinit(self,Zsk):
        #TODO
        Gt = np.asmatrix(np.eye(self.nphases))*.01
        return(Gt)


    def pfEqns3phase(self,VmagTarg,VangTarg,Zskest):
        error('not built')
        return


    def phasorI_estFromScmd(self, Vmag_relative, Vang, Pcmd, Qcmd):
        '''
        Vang is relative, so its base 0 for all phases (not +/- 2pi/3) so Vcomp will be will be deflected from (1,0)
        all entries of Scomp_est are deflected from (1,0)
        so all entries of Icomp_est will be deflected from (1,0)
        want the operations to be element_wise so pass in arrays not matrices
        '''
        Vcomp = Vmag_relative*np.cos(Vang) + Vmag_relative*np.sin(Vang)*1j #Vmag_relative is local - ref, so positive current/power flow is into the network: true because commands are positive for power injections
        Scomp_est = Pcmd + Qcmd*1j #this could be delta S to give delta I_est directly, but the same delta I_est is ultimately attained subtracting I_est_prev later on
        Icomp_est = np.conj(Scomp_est/Vcomp) #this will do element_wise computation bc they're arrays
        return (Icomp_est)


    def LQRupdate(self,Vmag,Vang,VmagTarg,VangTarg,VmagRef,VangRef,sat_arrayP,sat_arrayQ,Icomp=np.NaN):
        '''
        Internal Controller Accounting and feedback calculation
        Expects 1-d arrays
        set statek to Vmag and Vang and increment integrator
        Vang must be base 0 (ie not +/- 2pi/3), which it is bc its a relative angle
        and Icomp must be deflected from (1,0), which it is because its computed from relative angles
        all vectors are row vectors so they can be converted back into 1-d arrays easily
        '''
        if np.isnan(Icomp):
            Vmag_relative = Vmag - VmagRef
            Icomp_est = self.phasorI_estFromScmd(Vmag_relative, Vang, self.PcommandPrev, self.QcommandPrev) #this estimate should be valid even if there are other loads on the LPBC node (as long as the loads are uncorrelated with the commands)
            #HERE Vmag_relative cant be 0 bc then I est will be inf
            # Icomp_est = np.ones(self.nphases) #just for debugging when netowrk is powered down
            Icomp = np.asmatrix(Icomp_est)
            # print('Icompest : ' + str(Icomp))
        else:
            Icomp = np.asmatrix(Icomp)
        Vcomp = np.asmatrix(Vmag*np.cos(Vang) + Vmag*np.sin(Vang)*1j)
        Vmag = np.asmatrix(Vmag) #come in as 1-d arrays, asmatrix makes them single-row matrices (vectors)
        Vang = np.asmatrix(Vang)
        VmagTarg = np.asmatrix(VmagTarg)
        VangTarg = np.asmatrix(VangTarg)
        VmagRef = np.asmatrix(VmagRef)
        VangRef = np.asmatrix(VangRef)

        self.V0 = self.setVtarget(VmagTarg,VangTarg)
        self.setVref(VmagRef,VangRef)
        self.iteration_counter += 1
        if np.sum(sat_arrayP) + np.sum(sat_arrayQ) > 0: #these are 1 if unsaturated
            self.allsaturated = 0 #for turning off the integrator
        else:
            self.allsaturated = 1
        if np.sum(sat_arrayP) + np.sum(sat_arrayQ) < self.nphases*2: #these are 1 if unsaturated
            self.onesaturated = 1 #for turning off Zskest
        else:
            self.onesaturated = 0

        #Estimate Zeff
        if self.use_Zsk_est == 1 and (self.currentMeasExists == 1 or self.onesaturated == 0): #only run Zsk est if you have a current measurement or the actuators arent saturated
            if self.iteration_counter != 1: # There arent any previous measurements at t=1, so you cant update Zeff
                dtVt = (Vcomp - self.VcompPrev).T #these are vertical vectors
                dtIt = (Icomp - self.IcompPrev).T
                self.Gt = self.Gt/self.lam - (self.Gt*(dtIt*dtIt.H)*self.Gt)/(self.lam**2*(1 + dtIt.H*self.Gt*dtIt/self.lam))
                err = dtVt - self.Zskest*dtIt
                self.Zskest = np.asmatrix(self.Zskest.H + self.Gt*dtIt*err.H).H
                self.Zskest = (self.Zskest + self.Zskest.T)/2
            for p in np.arange(self.nphases):
                if np.real(self.Zskest[p,p]) < 0:
                    self.Zskest = self.Zskestinit
                    self.Gt = self.Gt*.1
                    print('reset impedance estimator')
            self.B = self.updateB(self.Zskest)

        if self.allsaturated:
            self.state = np.hstack((Vmag-self.VmagTarg,Vang-self.VangTarg,self.state[0,self.nphases*2:self.nphases*4]))
        else:
            self.state = np.hstack((Vmag-self.VmagTarg,Vang-self.VangTarg,self.timesteplength*self.state[0,0:self.nphases*2]+self.state[0,self.nphases*2:self.nphases*4]))

        #DOBC
        if self.linearizeplant:
            Babbrev = self.B[:self.nphases*2,:]
            ueff = np.linalg.pinv(Babbrev)*(np.hstack((Vmag,Vang))-self.V0).T
        else:
            ueff = self.pfEqns3phase(Vmag,Vang,self.Zskest) #havent built these yet #ueff through Zeff would give Vmeas
        if self.iteration_counter != 1: #iteration_counter is 1 in the first call
            dm = ueff.T - self.u #dm for d measurement
            self.d = (1-self.lpAlpha)*self.d + self.lpAlpha*dm
        else:
            dm = ueff
            self.d = dm

        #update uref
        if self.linearizeplant:
            uref = (np.linalg.pinv(Babbrev)*(np.hstack((self.VmagTarg,self.VangTarg))-self.V0).T).T
        else:
            uref = self.pfEqns3phase(VmagTarg,VangTarg,self.Zskest) #havent built these yet

        #Feedback Control input for next round
        self.u = (self.K*self.state.T).T + uref - self.d

        #save measurements for next round
        self.IcompPrev = Icomp.copy()
        self.VcompPrev = Vcomp.copy()
        if np.mod(self.iteration_counter-1,self.controllerUpdateCadence) == 0:
            self.K = self.updateController(self.A,self.B,self.Qcost,self.Rcost)

        #LQR defines u as positive for power flowing out of the network (due to the signs of the PF linearization)
        #but general LPBC convention is power is defined as postive into the network
        #returns powers in pu, I believe
        Plpbc = np.asarray(-self.u[0,0:self.nphases])[0]
        Qlpbc = np.asarray(-self.u[0,self.nphases:2*self.nphases])[0]
        self.PcommandPrev = Plpbc.copy() #used if no I measurement is available
        self.QcommandPrev = Qlpbc.copy()

        #sanity check for debugging
        # print('Zskest : ' + str(self.Zskest))
        # print('A : ' + str(self.A))
        # print('B : ' + str(self.B))
        # print('Babbrev : ' + str(Babbrev))
        # print('uref : ' + str(uref))
        # print('state' + str(self.state))
        # print('u : ' + str(self.u))
        # print('d : ' + str(self.d))
        # print('IcompPrev' + str(self.IcompPrev))
        # print('VcompPrev' + str(self.VcompPrev))
        # print('K : ' + str(self.K))
        # print('PcommandPrev : ' + str(self.PcommandPrev))
        # print('QcommandPrev : ' + str(self.QcommandPrev))

        return (Plpbc,Qlpbc)
