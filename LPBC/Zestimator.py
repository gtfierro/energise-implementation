
import numpy as np
from scipy.linalg import solve_discrete_are as dare
from numpy.linalg import inv

printZeffterms = 1

class Zestimator:
    def __init__(self,lpbcbus,nphases,Zeffkinit,useRefNode=False,useNominalV=False,currentMeasExists=1,lam=.99,Gt=None,controllerUpdateCadence=1,ZeffkinitInPU=1):
        self.lpbcbus = lpbcbus
        self.nphases = nphases
        self.V0mag = np.NaN
        self.V0ang = np.NaN
        self.V0 = np.NaN #np.hstack((self.V0mag,self.V0ang))
        self.VmagTarg = np.NaN
        self.VangTarg = np.NaN
        # self.timesteplength = timesteplength
        self.currentMeasExists = currentMeasExists
        self.LQR_stepcounter = 0

        #Controller Parameters
        self.controllerUpdateCadence = controllerUpdateCadence

        #Z estimator parameters
        # self.est_Zeffk = est_Zeffk #boolean
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
            # self.Gt = np.asmatrix(np.eye(self.nphases) + np.eye(self.nphases)*1j,dtype=np.complex_)*.01 #HERE
        else:
            self.Gt = Gt

        self.useRefNode = useRefNode
        self.useNominalV = useNominalV

        self.Babbrev = None #np.zeros((nphases,nphases))

        self.IcompPrev = np.asmatrix(np.zeros(nphases,dtype=np.complex_))
        self.VcompPrev = np.asmatrix(np.zeros(nphases,dtype=np.complex_))
        self.PcommandPrev = np.zeros(nphases)
        self.QcommandPrev = np.zeros(nphases)


####################################################

    def setV0(self,V0mag,V0ang):
        self.V0mag = np.asmatrix(V0mag)
        self.V0ang = np.asmatrix(V0ang)
        self.V0 = np.asmatrix(np.hstack((V0mag,V0ang)))
        return

    def setVtarget(self,VmagTarg,VangTarg):
        self.VmagTarg = np.asmatrix(VmagTarg)
        self.VangTarg = np.asmatrix(VangTarg)
        return

    def setZeffandZeffkestinitWnewZbase(self,Zbase,Zeffk_init_mult):
        ZeffkTru = self.ZeffkestinitNonPu/Zbase #ZeffkestTru is not stored by LQR controller bc the controlelr presumably doesnt know ZeffkestTru
        self.Zeffkestinit = ZeffkTru*Zeffk_init_mult
        self.Zeffkest = self.Zeffkestinit
        return self.Zeffkestinit, ZeffkTru

    def pointybracket(self, M):
        return np.asmatrix(np.vstack((np.hstack((np.real(M), -np.imag(M))),np.hstack((np.imag(M), np.real(M))))))
    def makeN(self, dimN): #dimN = nbuses*nphases*2
        return np.asmatrix(np.diag(np.hstack((np.ones(int(dimN/2)), -np.ones(int(dimN/2))))))

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
            print('Estimated Icomp for Zeffk estimation: ', Icomp_est)
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


    def getLinWRef(self, Zeffkest, Vcomp):
        #assumes injection-positive (rather than load (extraction)-positive)
        if self.useNominalV:
            alph = np.exp(-2/3*np.pi*1j)
            Phir = np.asmatrix(np.diag([1, alph, alph**2]))
            Phirinv = np.asmatrix(np.diag([1, alph**2, alph]))
            dimNR = self.nphases*2
            NR = self.makeN(dimNR)
            Babbrev = self.pointybracket(Phirinv*Zeffk*Phir)*NR
        # else:
        return Babbrev

    def getLinWoRef(self, Zeffkest, Vcomp, freq):
        # if self.useNominalV:
        # else:
        return self.Babbrev

    def ZeffandLinUpdate(self,VmagArray,VangArray,P_implemented=None,Q_implemented=None,V0magArray=None,V0angArray=None,freq=None,sat_arrayP=None,sat_arrayQ=None,VcompArray=None,IcompArray=None):
        '''
        Uses reference node measurement
        uses matrix computations internally, but gets ndarrays in and passes ndarrays out
        all vectors are row vectors so they can be converted back into 1-d arrays easily
        '''
        if P_implemented is not None: #before any P is implemented P_implemented = none and self.u = 0
            # self.u[0,:self.nphases] = P_implemented
            self.PcommandPrev = P_implemented
            print('&&&&&&&&&&&& self.PcommandPrev', self.PcommandPrev)
        else:
            print('&&&&&&&&&&&& P_implemented was None')
        if Q_implemented is not None:
            # self.u[0,self.nphases:] = Q_implemented
            self.QcommandPrev = Q_implemented
            print('&&&&&&&&&&&& self.QcommandPrev', self.QcommandPrev)
        else:
            print('&&&&&&&&&&&& Q_implemented was None')

        # self.LQR_stepcounter += 1
        if sat_arrayP is None:
            sat_arrayP = np.ones(self.nphases)
        if sat_arrayQ is None:
            sat_arrayQ = np.ones(self.nphases)

        #Convert Vs to np.matrices
        if VcompArray is not None:
            Vcomp = np.asmatrix(VcompArray)
        else:
            Vcomp = np.asmatrix(VmagArray*np.cos(VangArray) + VmagArray*np.sin(VangArray)*1j) #Vang has to include the 120 deg shifts
        Vmag = np.asmatrix(VmagArray) #come in as 1-d arrays, asmatrix makes them single-row matrices (vectors)
        Vang = np.asmatrix(VangArray)

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

        if self.useRefNode:
            assert V0mag != None and V0ang != None, 'Need to give V0 if using ref node'
            V0mag = np.asmatrix(V0magArray)
            V0ang = np.asmatrix(V0angArray)
            self.setV0(V0mag,V0ang)
            # self.Babbrev = getLinWRef(self, Zeffkest, Vcomp)
            self.Babbrev = None
        else:
            assert freq != None, 'Need to give freq if not using ref node'
            # self.Babbrev = getLinWoRef(self, Zeffkest, Vcomp, freq)
            self.Babbrev = None

        return self.Zeffkest, self.Gt, self.Babbrev
