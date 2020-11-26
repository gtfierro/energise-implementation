
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os

#HERE have to put the file names and path in manually
#The file names will also be the names in the legend
current_directory = os.getcwd()
simFolderPATH = os.path.join(current_directory, 'simulationPlots')
# resultsPATH = os.path.join(simFolderPATH, f'feeder:13bal_bus:675')
# fileNames = ['ZestData_eps=0.5',  'ZestData_eps=0.75',  'ZestData_eps=1.25', 'ZestData_eps=1.5', 'ZestData_eps=2']

resultsPATH = os.path.join(simFolderPATH, f'feeder:13bal_bus:675_Wcaps_5runs_perturb:p01_lam:p95_G:100')
fileNames = ['ZestData_λ=0.5',  'ZestData_λ=0.75',  'ZestData_λ=1.25', 'ZestData_λ=1.5', 'ZestData_λ=2']
labelNames = [r'$\epsilon$ = 0.5', r'$\epsilon$ = 0.75', r'$\epsilon$ = 1.25', r'$\epsilon$ = 1.5', r'$\epsilon$ = 2']

# resultsPATH = os.path.join(simFolderPATH, f'feeder:13bal_bus:675_WOcaps_5runs_perturb:p01_lam:p95_G:100')
# fileNames = ['ZestData_eps=0.5',  'ZestData_eps=0.75',  'ZestData_eps=1.25', 'ZestData_eps=1.5', 'ZestData_eps=2.0']
# labelNames = [r'$\epsilon$ = 0.5', r'$\epsilon$ = 0.75', r'$\epsilon$ = 1.25', r'$\epsilon$ = 1.5', r'$\epsilon$ = 2']

# resultsPATH = os.path.join(simFolderPATH, f'feeder:13bal_bus:675_WOcaps_5runsuniformRandom_perturb:p01_lam:p95_G:100')
# fileNames = ['ZestData_eps=uniRandom1.0',  'ZestData_eps=uniRandom2.0',  'ZestData_eps=uniRandom3.0', 'ZestData_eps=uniRandom4.0', 'ZestData_eps=uniRandom5.0']
# labelNames = [r'Random Initialization 1', r'Random Initialization 2', r'Random Initialization 3', r'Random Initialization 4', r'Random Initialization 5']

# resultsPATH = os.path.join(simFolderPATH, f'feeder:13bal_bus:675_WOcaps_5runs_localPhasors_perturb:p01_lam:p95_G:100')
# fileNames = ['ZestData_eps=0.5',  'ZestData_eps=0.75',  'ZestData_eps=1.25', 'ZestData_eps=1.5', 'ZestData_eps=2.0']
# labelNames = [r'$\epsilon$ = 0.5', r'$\epsilon$ = 0.75', r'$\epsilon$ = 1.25', r'$\epsilon$ = 1.5', r'$\epsilon$ = 2']

plotZest = 1
subplots = 1

plotVmag = 0
plotVmag_noPerturbToo = 0
VmagIndex = 1

if subplots:
    fig, (ax1, ax2), = plt.subplots(1,2, figsize=(10,5))
    if plotZest:
        colors = ['olivedrab', 'teal', 'darkmagenta', 'slateblue', 'darkorange']
        for i in np.arange(len(fileNames)):
            print('fileNames[i] ', fileNames[i])
            Z_df = pd.read_csv(os.path.join(resultsPATH, f'{fileNames[i]}.csv'), index_col=0)
            Z_array = Z_df[['Zth Estimation Error']].to_numpy()
            ax1.plot(Z_array, label=labelNames[i], color=colors[i]) # this uses the filename for the legend
        # ax1.title(r'$Z_{Th}$ Estimation Error')
        # ax1.ylabel('Frobenius Norm Error')
        ax1.set(ylabel=r'$Z_{Th}$ Frobenius Norm Error')
        ax1.set(xlabel='Timestep')
        ax1.legend()
        # plt.show()

        for i in np.arange(len(fileNames)):
            Z_df = pd.read_csv(os.path.join(resultsPATH, f'{fileNames[i]}.csv'), index_col=0)
            Z_array = Z_df[['Gt']].to_numpy()
            ax2.plot(Z_array, label=labelNames[i], color=colors[i]) # this uses the filename for the legend
        # plt.title('$F$ Magnitude')
        ax2.set(ylabel='Frobenius Norm of $F$')
        ax2.set(xlabel='Timestep')
        # ax2.legend()

        plt.tight_layout()
        plt.show()

else:
    if plotZest:
        colors = ['darkorange', 'olivedrab', 'teal', 'maroon', 'slateblue']
        for i in np.arange(len(fileNames)):
            print('fileNames[i] ', fileNames[i])
            Z_df = pd.read_csv(os.path.join(resultsPATH, f'{fileNames[i]}.csv'), index_col=0)
            Z_array = Z_df[['Zth Estimation Error']].to_numpy()
            plt.plot(Z_array, label=labelNames[i], color=colors[i]) # this uses the filename for the legend
        # plt.title(r'$Z_{Th}$ Estimation Error')
        # plt.ylabel('Frobenius Norm Error')
        plt.ylabel(r'$Z_{Th}$ Frobenius Norm Error')
        plt.xlabel('Timestep')
        plt.legend()
        plt.show()

        for i in np.arange(len(fileNames)):
            Z_df = pd.read_csv(os.path.join(resultsPATH, f'{fileNames[i]}.csv'), index_col=0)
            Z_array = Z_df[['Gt']].to_numpy()
            plt.plot(Z_array, label=labelNames[i], color=colors[i]) # this uses the filename for the legend
        # plt.title('$F$ Magnitude')
        plt.ylabel('Frobenius Norm of $F$')
        plt.xlabel('Timestep')
        plt.legend()
        plt.show()

if plotVmag:
    nodeNames = ['Va Magnitude', 'Vb Magnitude', 'Vc Magnitude']
    Z_df = pd.read_csv(os.path.join(resultsPATH, f'{fileNames[VmagIndex]}.csv'), index_col=0)
    Z_array = Z_df[['Va Magnitude', 'Vb Magnitude', 'Vc Magnitude']].to_numpy()
    plt.plot(Z_array[:,0], label='phase A', color='blue')# with perturbation')
    plt.plot(Z_array[:,1], label='phase B', color='orangered')# with perturbation')
    plt.plot(Z_array[:,2], label='phase C', color='green')# with perturbation')
        # if i == 0:
        #     plt.plot(Z_array, label='phase A') # this uses the filename for the legend
        # elif i == 1:
        #     plt.plot(Z_array, label='phase B') # this uses the filename for the legend
        # else:
        #     plt.plot(Z_array, label='phase C') # this uses the filename for the legend
    # plt.title('Node 675 Voltage Magnitude')
    plt.ylabel('Voltage Magnitude [p.u.]')
    plt.xlabel('Timestep')
    plt.legend()

    if plotVmag_noPerturbToo:
        nodeNames = ['Va Magnitude', 'Vb Magnitude', 'Vc Magnitude']
        Z_df = pd.read_csv(os.path.join(resultsPATH, f'ZestData_noPerturb.csv'), index_col=0)
        Z_array = Z_df[['Va Magnitude', 'Vb Magnitude', 'Vc Magnitude']].to_numpy()
        plt.plot(Z_array[:,0], '--', color='blue')#, label='phase A without perturbation')
        plt.plot(Z_array[:,1], '--', color='orangered')#, label='phase B without perturbation')
        plt.plot(Z_array[:,2], '--', color='green')#, label='phase C without perturbation')
            # if i == 0:
            #     plt.plot(Z_array, label='phase A') # this uses the filename for the legend
            # elif i == 1:
            #     plt.plot(Z_array, label='phase B') # this uses the filename for the legend
            # else:
            #     plt.plot(Z_array, label='phase C') # this uses the filename for the legend
        # plt.title('Node 675 Voltage Magnitude')
        plt.ylabel('Voltage Magnitude [p.u.]')
        plt.xlabel('Timestep')
        plt.legend(loc='upper right')

    plt.show()

# ['Zth Estimation Error', 'Gt', 'Va Magnitude', 'Vb Magnitude', 'Vc Magnitude']
