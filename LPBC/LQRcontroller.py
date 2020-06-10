
import numpy as np
from scipy.linalg import solve_discrete_are as dare
from numpy.linalg import inv

printZeffterms = 1
printDOBCterms = 1
printControlterms = 1

class LQRcontroller:
    def __init__(self,lpbcbus,nphases,timesteplength,Qcost,Rcost,Zeffkinit,est_Zeffk=0,cancelDists=1,currentMeasExists=1,lpAlpha=.1,lam=.99,Gt=None,controllerUpdateCadence=1,linearizeplant=1,ZeffkinitInPU=1):
        self.lpbcbus = lpbcbus
        self.nphases = nphases
        self.V0mag = np.NaN
        self.V0ang = np.NaN
        self.V0 = np.NaN #np.hstack((self.V0mag,self.V0ang))
        self.VmagTarg = np.NaN
        self.VangTarg = np.NaN
        self.timesteplength = timesteplength
        self.currentMeasExists = currentMeasExists
        self.iteration_counter = 0

        #Controller Parameters
        self.Qcost = np.asmatrix(Qcost)
        self.Rcost = np.asmatrix(Rcost)
        self.controllerUpdateCadence = controllerUpdateCadence

        #DOBC parameters
        self.cancelDists = cancelDists
        self.lpAlpha = lpAlpha # low pass filter alpha, larger changes disturbance faster (more noise sensitive)
        self.linearizeplant = linearizeplant

        #Z estimator parameters
        self.est_Zeffk = est_Zeffk #boolean
        self.lam = lam # 0 < lam < 1, smaller lam changes Zeffkest faster
        self.dtItThreshold = 1e-4 # 1e-3 #could pass these thresholds in when you make the controller
        self.dtVtThreshold = 1e-4 # 1e-3
        self.powerThresholdForIcompEst = 1e-4 # 1e-3 #this is different than self.dtItThreshold. Idea is to prevent noisy Icom estiamtes when there isnt an S command. Downside is that you lose a useful measurement when S command goes to zero from a nonzero value. Could fix that case with an if statement, though.
        if ZeffkinitInPU:
            self.Zeffkestinit = np.asmatrix(Zeffkinit) #Zeffkinit comes in as a complex-valued array
            self.Zeffkest = np.asmatrix(Zeffkinit)
        else: #waits for setZeffandZeffkestinitWnewZbase to be called to set self.Zeffkestinit and self.Zeffkest
            self.ZeffkestinitNonPu = np.asmatrix(Zeffkinit)
        self.IcompPrevExists = 0
        if Gt is None: #done bc its bad to initialize a variable to a mutable type https://opensource.com/article/17/6/3-things-i-did-wrong-learning-python
            self.Gt = np.asmatrix(np.eye(self.nphases))*.01
            # self.Gt = np.asmatrix(np.eye(self.nphases) + np.eye(self.nphases)*1j,dtype=np.complex_)*.01 #HHERE
        else:
            self.Gt = Gt

        #Controller and state initialization
        if ZeffkinitInPU:
            (self.A, self.B, self.Babbrev) = self.makeABmatrices(Zeffkinit,timesteplength)
            self.K = self.updateController(self.A,self.B,self.Qcost,self.Rcost)
        self.state = np.asmatrix(np.zeros(4*nphases))
        self.u = np.asmatrix(np.zeros(2*nphases)) #this makes a row matrix

        #disturbance initialization
        self.d_est = np.asmatrix(np.zeros(2*nphases))
        self.IcompPrev = np.asmatrix(np.zeros(nphases,dtype=np.complex_))
        self.VcompPrev = np.asmatrix(np.zeros(nphases,dtype=np.complex_))
        self.PcommandPrev = np.zeros(nphases)
        self.QcommandPrev = np.zeros(nphases)


####################################################

    def setV0(self,V0mag,V0ang):
        self.V0mag = V0mag
        self.V0ang = V0ang
        self.V0 = np.hstack((V0mag,V0ang))
        return


    def setVtarget(self,VmagTarg,VangTarg):
        self.VmagTarg = VmagTarg
        self.VangTarg = VangTarg
        return


    def setZeffandZeffkestinitWnewZbase(self,Zbase,Zeffk_init_mult):
        ZeffkTru = self.ZeffkestinitNonPu/Zbase #ZeffkestTru is not stored by LQR controller bc the controlelr presumably doesnt know ZeffkestTru
        self.Zeffkestinit = ZeffkTru*Zeffk_init_mult
        self.Zeffkest = self.Zeffkestinit
        # (self.A, self.B, self.Babbrev) = self.makeABmatrices(self.Zeffkest,self.timesteplength)
        (self.A, self.B, self.Babbrev) = self.makeABmatrices(np.asarray(self.Zeffkest),self.timesteplength) #not sure the asarray is necessary..
        self.K = self.updateController(self.A,self.B,self.Qcost,self.Rcost)
        return self.Zeffkestinit, ZeffkTru


    def makeABmatrices(self,Zeffk,timesteplength):
        A = np.vstack((np.zeros((2*self.nphases,4*self.nphases)),np.hstack((timesteplength*np.eye(2*self.nphases),np.eye(2*self.nphases)))))
        A = np.asmatrix(A)
        B, Babbrev = self.getB(Zeffk)
        return A, B, Babbrev


    def pointybracket(self, M):
        return np.asmatrix(np.vstack((np.hstack((np.real(M), -np.imag(M))),np.hstack((np.imag(M), np.real(M))))))
    def makeN(self, dimN): #dimN = nbuses*nphases*2
        return np.asmatrix(np.diag(np.hstack((np.ones(int(dimN/2)), -np.ones(int(dimN/2))))))


    def getB(self, Zeffk):
        #assumes injection-positive (rather than load (axtraction)-positive)
        alph = np.exp(-2/3*np.pi*1j)
        Phir = np.asmatrix(np.diag([1, alph, alph**2]))
        Phirinv = np.asmatrix(np.diag([1, alph**2, alph]))
        dimNR = self.nphases*2
        NR = self.makeN(dimNR)
        Babbrev = self.pointybracket(Phirinv*Zeffk*Phir)*NR
        B = np.vstack((Babbrev, np.asmatrix(np.zeros((2*self.nphases,2*self.nphases)))))
        return B, Babbrev


    def updateController(self,A,B,Q,R):
        S = np.asmatrix(dare(A,B,Q,R)) #from scipy
        K = -(R+B.T*S*B).I*B.T*S*A # neg for neg feedback
        return K


    def calcGt_And_Zeffkinit(self,Zeffk):
        #TODO
        Gt = np.asmatrix(np.eye(self.nphases))*.01
        return(Gt)


    def pfEqns3phase(self,Vmag,Vang,Zeffk):
        V = np.multiply(Vmag,np.cos(Vang)) + np.multiply(Vmag,np.sin(Vang))*1j
        V0 = np.multiply(self.V0mag,np.cos(self.V0ang)) + np.multiply(self.V0mag,np.sin(self.V0ang))*1j
        dV = V - V0 #injection-positive
        I = np.linalg.pinv(Zeffk)*dV.T
        S = np.multiply(V.T,np.conj(I))
        PQrowmat = np.asmatrix(np.hstack((np.real(S.T),np.imag(S.T))))
        return PQrowmat #return as a row matrix bc thats what LQR update uses


    def calc_u_naive(self,VmagTarg,VangTarg): #built this for debugging
        VmagTarg = np.asmatrix(VmagTarg)
        VangTarg = np.asmatrix(VangTarg)
        if self.linearizeplant:
            u_naive = (np.linalg.pinv(self.Babbrev)*(np.hstack((VmagTarg,VangTarg))-self.V0).T).T
        else:
            u_naive = self.pfEqns3phase(VmagTarg,VangTarg,self.Zeffkest)
        return u_naive


    def updateDisturbance(self,Vmag,Vang,printDOBCterms):
        if self.linearizeplant:
            u_d_eff = (np.linalg.pinv(self.Babbrev)*(np.hstack((Vmag,Vang))-self.V0).T).T # S~=Y*dV. dV is the difference from the substation (ref) voltage, V0
        else:
            u_d_eff = self.pfEqns3phase(Vmag,Vang,self.Zeffkest)
        if self.iteration_counter != 1: #iteration_counter is 1 in the first call
            dm = u_d_eff - self.u
            self.d_est = (1-self.lpAlpha)*self.d_est + self.lpAlpha*dm
        else: #initialize
            dm = u_d_eff
            self.d_est = dm
        if printDOBCterms:
            print('u_d_eff ' + str(u_d_eff))
            print('self.d_est ' + str(self.d_est))
        return self.d_est #returning this is redundant


    def phasorI_estFromScmd(self, Vcomp, Pcmd, Qcmd):
        #I is injection-positive
        Scomp_est = Pcmd + Qcmd*1j #this could be delta S to give delta I_est directly, but the same delta I_est is ultimately attained subtracting I_est_prev later on
        Icomp_est = np.conj(Scomp_est/Vcomp) #this will do element_wise computation bc they're arrays
        return Icomp_est


    def getIcomp(self,IcompArray,Vcomp): #get Icomp meas as np.matrix
        if self.currentMeasExists:
            if any(np.isnan(IcompArray)):
                print('``````````````````````````````````````````````````````````````````')
                print(f'One of the current measurements was a nan')
                Icomp = None
            else:
                Icomp = np.asmatrix(IcompArray)
        elif self.onesaturated == 0 and all(np.abs(self.PcommandPrev) > self.powerThresholdForIcompEst) and all(np.abs(self.QcommandPrev) > self.powerThresholdForIcompEst):
            # the actuators arent saturated (so the estimation of the current meas from the power command is legit)
            Icomp_est = self.phasorI_estFromScmd(Vcomp, self.PcommandPrev, self.QcommandPrev)
            Icomp = np.asmatrix(Icomp_est)
        else:
            if printZeffterms:
                print('``````````````````````````````````````````````````````````````````')
                print(f'Not providing an Icomp this round because')
                if self.onesaturated == 1:
                    print(f'one actuator is saturated, self.onesaturated: {self.onesaturated}')
                else:
                    print(f'the power commands were too small, self.PcommandPrev: {self.PcommandPrev}, self.QcommandPrev: {self.QcommandPrev}')
            Icomp = None
        return Icomp


    def updateZeff(self,Vcomp,Icomp=None):
        dtVt = (Vcomp - self.VcompPrev).T #these are vertical vectors
        dtIt = (Icomp - self.IcompPrev).T
        if all(abs(dtIt) > self.dtItThreshold) and all(abs(dtVt) > self.dtVtThreshold):
            self.Gt = self.Gt/self.lam - (self.Gt*(dtIt*dtIt.H)*self.Gt)/(self.lam**2*(1 + dtIt.H*self.Gt*dtIt/self.lam))
            err = dtVt - self.Zeffkest*dtIt
            self.Zeffkest = np.asmatrix(self.Zeffkest.H + self.Gt*dtIt*err.H).H
            self.Zeffkest = (self.Zeffkest + self.Zeffkest.T)/2
            if printZeffterms:
                print('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
                print(f'Zeffkest update for lpbc {self.lpbcbus}')
                print(f'lam {self.lam}')
                print(f'Gt {self.Gt}')
                print(f'err {err}')
                print(f'Zeffkest : {self.Zeffkest}')
            for p in np.arange(self.nphases):
                if np.real(self.Zeffkest[p,p]) < 0:
                    self.Zeffkest = self.Zeffkestinit
                    self.Gt = self.Gt*.1
                    print('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
                    print('reseting impedance estimator, reducing the size of Gt')
        else:
            if printZeffterms:
                print('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
                print(f'Not updating Zeffkest this round because')
                if any(abs(dtIt) < self.dtItThreshold):
                    print(f'dtIt is too small. abs(dtIt) = {abs(dtIt)}')
                elif any(abs(dtVt) < self.dtVtThreshold): #would be surprised if this got trigerred and should track down why
                    print(f'dtVt is too small. abs(dtVt) = {abs(dtVt)}')
                else:
                    print(f'something else. dtIt: {dtIt}, dtVt: {dtVt}')
                print(f'Zeffkest update for lpbc {self.lpbcbus}')
                print(f'lam {self.lam}')
                print(f'Gt {self.Gt}')
                print(f'Zeffkest : {self.Zeffkest}')
            self.Zeffkest = self.Zeffkest
            self.Gt = self.Gt
        return self.Zeffkest, self.Gt

    '''
    Vcomp calc (needs to be base [0,0,0], and therefore ~[0,-120,120])
    state: diff bn V and Vtarg
    unaive: diff bn Vtarg and V0 if linearize plant is used (trickier if pfEqns3phase is used)
    d_m: diff bn Vtarg and V0 if linearize plant is used (trickier if pfEqns3phase is used)

    so if V and Vtarg are relative, and V0 is ~[0,0,0] (instead of ~[0,-120,120]), and Vcomp is set seperately to be ~[0,-120,120] for the Zest calc,
    then it should work to pass LQR relative targets and V0 =~ [0,0,0]
    '''
    # def LQRupdate(self,VmagArray,VangArray,VmagTargArray,VangTargArray,V0magArray,V0angArray,sat_arrayP=None,sat_arrayQ=None,VcompArray=None,IcompArray=None):
    def LQRupdate(self,VmagArray,VangArray,VmagTargArray,VangTargArray,V0magArray,V0angArray,P_implemented=None,Q_implemented=None,sat_arrayP=None,sat_arrayQ=None,VcompArray=None,IcompArray=None):
        '''
        Internal Controller Accounting, feedback calculation, and Z estimation (if self.est_Zeffk was set to 1)
        LQR uses matrix computations internally, but gets ndarrays in and passes ndarrays out
        all vectors are row vectors so they can be converted back into 1-d arrays easily
        '''
        # print('~~~~~~~~~~ HERE u before ~~~~~~~~~~~~~~ ', self.u)
        if P_implemented is not None: #before any P is implemented P_implemented = none and self.u = 0
            self.u[0,:self.nphases] = P_implemented
            self.PcommandPrev = P_implemented
        if Q_implemented is not None:
            self.u[0,self.nphases:] = Q_implemented
            self.QcommandPrev = Q_implemented
        # print('````````` HERE u after `````````````` ', self.u)

        self.iteration_counter += 1
        if sat_arrayP is None:
            sat_arrayP = np.ones(self.nphases)
        if sat_arrayQ is None:
            sat_arrayQ = np.ones(self.nphases)

        #Convert Vs to mp.matrices
        if VcompArray is not None:
            Vcomp = np.asmatrix(VcompArray)
        else:
            Vcomp = np.asmatrix(VmagArray*np.cos(VangArray) + VmagArray*np.sin(VangArray)*1j) #Vang has to include the 120 deg shifts
        Vmag = np.asmatrix(VmagArray) #come in as 1-d arrays, asmatrix makes them single-row matrices (vectors)
        Vang = np.asmatrix(VangArray)
        VmagTarg = np.asmatrix(VmagTargArray)
        VangTarg = np.asmatrix(VangTargArray)
        V0mag = np.asmatrix(V0magArray)
        V0ang = np.asmatrix(V0angArray)

        #set self.V0 and self.Vtarget
        self.setVtarget(VmagTarg,VangTarg)
        self.setV0(V0mag,V0ang)

        #check allsaturated and onesaturated
        if np.sum(sat_arrayP) + np.sum(sat_arrayQ) > 0: #these are 1 if unsaturated
            self.allsaturated = 0 #for turning off the integrator
        else:
            self.allsaturated = 1
        if np.sum(sat_arrayP) + np.sum(sat_arrayQ) < self.nphases*2: #these are 1 if unsaturated
            self.onesaturated = 1 #for turning off Zeffkest
        else:
            self.onesaturated = 0

        #Estimate Zeff
        if self.est_Zeffk == 1:
            Icomp = self.getIcomp(IcompArray, Vcomp) #Vcomp only necessary if self.currentMeasExists == 0
            if Icomp is not None:  #only run Zeffk est if you have a current measurement or a legit estimate
                if self.IcompPrevExists:
                    self.Zeffkest, self.Gt = self.updateZeff(Vcomp,Icomp)
                self.IcompPrev = Icomp.copy()
                self.VcompPrev = Vcomp.copy()
                self.IcompPrevExists = 1
            else:
                print('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
                print('Icomp is None')
                self.IcompPrevExists = 0

        #update controller feedback matrix
        if self.est_Zeffk == 1 and np.mod(self.iteration_counter-1,self.controllerUpdateCadence) == 0:
            self.B, self.Babbrev  = self.getB(self.Zeffkest)
            self.K = self.updateController(self.A,self.B,self.Qcost,self.Rcost)

        #calculate new state
        if self.allsaturated: #dont increment integrator states
            self.state = np.hstack((Vmag-self.VmagTarg,Vang-self.VangTarg,self.state[0,self.nphases*2:self.nphases*4]))
        else: #increment integrator states
            self.state = np.hstack((Vmag-self.VmagTarg,Vang-self.VangTarg,self.timesteplength*self.state[0,0:self.nphases*2]+self.state[0,self.nphases*2:self.nphases*4]))

        #DOBC
        if self.cancelDists:
            self.d_est = self.updateDisturbance(Vmag,Vang,printDOBCterms)

        #Feedback Control input for next round
        if self.linearizeplant:
            u_naive = (np.linalg.pinv(self.Babbrev)*(np.hstack((self.VmagTarg,self.VangTarg))-self.V0).T).T
        else:
            u_naive = self.pfEqns3phase(VmagTarg,VangTarg,self.Zeffkest)
        #LQR feedback control
        u_fb = (self.K*self.state.T).T
        if self.cancelDists:
            self.u = u_fb + u_naive - self.d_est
        else:
            self.u = u_fb + u_naive
        if printControlterms:
            print('LQR state: ' + str(self.state))
            # print('self.K: ' + str(self.K))
            print('LQR output u_fb: ' + str(u_fb))
            print('u_naive (S extraction that would naively give Vtarg - V0): ' + str(u_naive))
            print('self.u (self.u = u_fb + u_naive - self.d_est): ' + str(self.u))

        #general LPBC convention is power is defined as positive into the network (now, since I got changed the B matrix)
        #returns powers in pu if impedance and voltages are given in pu
        Pinj = np.asarray(self.u[0,0:self.nphases])[0] #convert from matrix back to array
        Qinj = np.asarray(self.u[0,self.nphases:2*self.nphases])[0]
        self.PcommandPrev = Pinj.copy() #commandPrevs are used if no I measurement is available
        self.QcommandPrev = Qinj.copy()

        return Pinj, Qinj, self.Zeffkest, self.Gt
