# -*- coding: utf-8 -*-

import sys, os
#sys.path.append(os.pardir)
sys.path.append(os.path.join(os.pardir, os.pardir))

import argparse 
import csv
import numpy as np
import matplotlib.pyplot as plt

from reference_path import ArcReferencePath



if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", default=2, type=int)             # Sets PyTorch and Numpy seeds
    parser.add_argument("--subplots", default="hor")               # Subplots direction ("hor", "ver")
    args = parser.parse_args() 

    # read results of pure pursuit
    #pp_file = 'log_pp_20231115_2107.csv' 
    pp_file = 'log_pp_20231123_2028.csv' 
    
    pp_x = []
    pp_y = []
    pp_cte = []
    pp_vr = []
    pp_v = []
    pp_wr = []
    pp_w = []
    pp_s = []
    
    with open(pp_file, 'r') as csv_file:
        csv_reader = csv.reader(csv_file)
        next(csv_reader)  # 跳过标题行
        for row in csv_reader:
            pp_x.append(float(row[1]))
            pp_y.append(float(row[2]))
            pp_cte.append(float(row[4]))
            pp_vr.append(float(row[6]))
            pp_v.append(float(row[7]))
            pp_wr.append(float(row[8]))
            pp_w.append(float(row[9]))
            pp_s.append(float(row[10]))
    
            
    # compute RMSE of CTE
    cte_pow = [x * x for x in pp_cte[:-8]]
    rmse = np.sqrt(np.sum(cte_pow) / len(pp_cte[:-8]))
    print(f"*RMSE: {rmse:.4f}") 
    print(f"*MAX: {max(np.abs(pp_cte)):.4f}") 
    print(f"*avg_vel: {np.mean(pp_v[1:-8]):.4f}")
    print("---------------------------------------")
            
    # read results of sac
    if args.seed == 0:
        sac_file = 'log_sac0_20231115_2053.csv' 
    elif args.seed == 1:
        sac_file = 'log_sac1_20231115_2056.csv'  
    elif args.seed == 2:
        sac_file = 'log_sac2_20231115_2059.csv' 
    elif args.seed == 3:
        sac_file = 'log_sac3_20231115_2101.csv'  
    elif args.seed == 4:
        sac_file = 'log_sac4_20231115_2103.csv'  
    
    sac_t = []
    sac_x = []
    sac_y = []
    sac_cte = []
    sac_vr = []
    sac_v = []
    sac_wr = []
    sac_w = []
    sac_s = []
    
    with open(sac_file, 'r') as csv_file:
        csv_reader = csv.reader(csv_file)
        next(csv_reader)  # skip title row
        for row in csv_reader:
            sac_t.append(float(row[0]))
            sac_x.append(float(row[1]))
            sac_y.append(float(row[2]))
            sac_cte.append(float(row[4]))
            sac_vr.append(float(row[6]))
            sac_v.append(float(row[7]))
            sac_wr.append(float(row[8]))
            sac_w.append(float(row[9]))
            sac_s.append(float(row[10]))

    
    # compute RMSE of CTE
    cte_pow = [x * x for x in sac_cte[:-8]]
    rmse = np.sqrt(np.sum(cte_pow) / len(sac_cte[:-8]))
    print(f"*RMSE: {rmse:.4f}") 
    print(f"*MAX: {max(np.abs(sac_cte)):.4f}") 
    print(f"*avg_vel: {np.mean(sac_v[1:-8]):.4f}")
    print("---------------------------------------")    
    
    """
    PLOTTING trajectories and cte
    """ 
    # create reference path
    path = ArcReferencePath("eight")
    s = np.linspace(0.0, path.len, 300)
    rx, ry = path.X(s), path.Y(s)
    
    # subplots 1 x 2
    if args.subplots == "hor":
        fig, ax = plt.subplots(1, 2, figsize=(13., 5.5))
        plt.subplots_adjust(wspace=0.3, top=0.9, bottom=0.13, left=0.085, right=0.99)
        ax[0].text(-0.14, 1.12, '(a)', transform=ax[0].transAxes, fontsize=18, va='top', ha='right')
        ax[1].text(-0.175, 1.12, '(b)', transform=ax[1].transAxes, fontsize=18, va='top', ha='right')

    # subplots 2 x 1
    if args.subplots == "ver":
        fig, ax = plt.subplots(2, 1, figsize=(8, 10))
        plt.subplots_adjust(hspace=0.25, top=0.89, bottom=0.065, left=0.15, right=0.94)

    plt.rcParams["font.size"] = 18
    plt.style.use('classic')
    fig.patch.set_alpha(0.0)

    ax[0].plot(rx, ry, linestyle=(5, (10, 3)), color="k", lw=2.5, label="Reference")
    ax[0].plot(pp_x, pp_y, color='b', lw=2.5, label="Pure Pursuit")
    ax[0].plot(sac_x, sac_y, color='r', lw=2.5, label="PP + SAC")
    ax[0].set_xlim(-1.2, 1.2)
    ax[0].set_ylim(-0.6, 0.6)
    ax[0].grid(True)
    ax[0].set_xlabel("x [m]", fontsize=18)
    ax[0].set_ylabel("y [m]", fontsize=18)
    ax[0].tick_params(axis='both', labelsize=18)
    legend = ax[0].legend(loc='upper center', bbox_to_anchor=(0.5, 1.15), ncol=1, fontsize=18)
    legend.get_frame().set_alpha(0.5)

    ax[1].plot(pp_s, pp_cte, color='b', lw=2.5, label="Pure Pursuit")
    ax[1].plot(sac_s, sac_cte, color='r', lw=2.5, label="PP + SAC")
    ax[1].set_xlim(pp_s[0], pp_s[-1])
    ax[1].set_ylim(min(pp_cte)-0.01, max(pp_cte)+0.01)
    ax[1].grid(True)
    ax[1].set_xlabel("Parameter $\lambda$", fontsize=18)
    ax[1].set_ylabel("cross-track error [m]", fontsize=18)
    ax[1].tick_params(axis='both', labelsize=18)
    legend = ax[1].legend(loc='upper right', ncol=1, fontsize=18)
    legend.get_frame().set_alpha(0.5)

    plt.show()
    plt.savefig('exp_eight_no_scale.jpg', format='jpg', dpi=300, bbox_inches='tight', pad_inches=0.05)    
        
    """
    PLOTTING velocities
    """ 
    # subplot 1 x 2
    if args.subplots == "hor":
        fig, ax = plt.subplots(1, 2, figsize=(13., 5.))
        plt.subplots_adjust(wspace=0.3, top=0.9, bottom=0.13, left=0.085, right=0.99)
        ax[0].text(-0.1, 1.12, '(a)', transform=ax[0].transAxes, fontsize=18, va='top', ha='right')
        ax[1].text(-0.14, 1.12, '(b)', transform=ax[1].transAxes, fontsize=18, va='top', ha='right')

    # subplots 2 x 1
    if args.subplots == "ver":
        fig, ax = plt.subplots(2, 1, figsize=(8, 10))
        plt.subplots_adjust(hspace=0.25, top=0.89, bottom=0.065, left=0.15, right=0.94)
    
    #ax[0].plot(pp_s, pp_vr, "--", color='k', lw=2.5, label="PP CMD")
    ax[0].plot(pp_s, pp_v, color='b', lw=2.5, label="Pure Pursuit")
    #ax[0].plot(sac_s, sac_vr, linestyle=(3, (9, 3)), color='k', lw=2.5, label="SAC CMD")
    ax[0].plot(sac_s, sac_v, color='r', lw=2.5, label="PP + SAC")
    ax[0].set_xlim(pp_s[0], pp_s[-1])
    ax[0].set_ylim(0.0, 0.45)
    ax[0].grid(True)
    ax[0].set_xlabel("Parameter $\lambda$", fontsize=18)
    ax[0].set_ylabel("Linear Velocity [m/s]", fontsize=18)
    ax[0].set_yticks([0.0, 0.1, 0.2, 0.3, 0.4])
    ax[0].tick_params(axis='both', labelsize=18)
    legend = ax[0].legend(loc='lower right', ncol=1, fontsize=18)
    legend.get_frame().set_alpha(0.5)
    
    #ax[1].plot(pp_s, pp_wr, "--", color='k', lw=2.5, label="PP CMD")
    ax[1].plot(pp_s, pp_w, color='b', lw=2.5, label="Pure Pursuit")
    #ax[1].plot(sac_s, sac_wr, linestyle=(3, (9, 3)), color='k', lw=2.5, label="SAC CMD")
    ax[1].plot(sac_s, sac_w, color='r', lw=2.5, label="PP + SAC")
    ax[1].set_xlim(pp_s[0], pp_s[-1])
    ax[1].set_ylim(-1.1, 1.1)
    ax[1].grid(True)
    ax[1].set_xlabel("Parameter $\lambda$", fontsize=18)
    ax[1].set_ylabel("Angular Velocity [rad/s]", fontsize=18)
    ax[1].tick_params(axis='both', labelsize=18)
    legend = ax[1].legend(loc='upper left', ncol=1, fontsize=18)
    legend.get_frame().set_alpha(0.5)
    plt.show()
    plt.savefig('exp_eight_vel_no_scale.jpg', format='jpg', dpi=300, bbox_inches='tight', pad_inches=0.05)
    
    """
    PLOTTING accelerations
    """ 
    plt.rcParams["font.size"] = 18
    fig, ax = plt.subplots(1, figsize=(8, 5))
    plt.style.use('classic')
    fig.patch.set_alpha(0.0)
    plt.subplots_adjust(hspace=0.2, top=0.96, bottom=0.125, left=0.15, right=0.94)
    ax.plot(sac_s, np.concatenate(([0], np.diff(sac_v)/np.diff(sac_t))), color='r', lw=2.5, label="identity")
    # vel diff.
    ax.set_xlim(sac_s[0], sac_s[-1])
    ax.set_ylim(-0.7, 0.4)
    plt.axhline(y=-0.5, color='k', linestyle='--', lw=2.5)
    plt.axhline(y=0.3, color='k', linestyle='--', lw=2.5)
    ax.grid(True)
    ax.set_xlabel("Parameter $\lambda$", fontsize=18)
    ax.set_ylabel("Linear Acceleration [m/s$^2$]", fontsize=18)
    #ax.set_yticks([0.0, 0.1, 0.2, 0.3, 0.4])
    ax.tick_params(axis='both', labelsize=18)        
    #ax2.legend(loc='lower right', ncol=1, fontsize=18)
    ax.figure.savefig('exp_eight_vel_diff_no_scale.jpg', format='jpg', dpi=300, bbox_inches='tight', pad_inches=0.05)
    plt.show()
     
    """
    PLOTTING linear velocity on trajectory
    """ 
    plt.rcParams["font.size"] = 18
    fig, ax = plt.subplots(1, figsize=(9, 5))
    plt.style.use('classic')
    fig.patch.set_alpha(0.0)
    plt.subplots_adjust(wspace=0.4, bottom=0.15, top=0.9)

    # create a scatterplot, setting the colormap to 'jet'
    ax.plot(rx, ry, linestyle=(5, (10, 3)), color="k", lw=2.5, label="Reference")
    sc = plt.scatter(sac_x, sac_y, s=80, c=sac_v, edgecolors='face', cmap='jet')

    # colorbar
    cbar = plt.colorbar(sc)
    cbar.ax.tick_params(labelsize=18) 
    cbar.set_label('Velocity Values [m/s]', fontsize=18)

    ax.set_xlim(-1.2, 1.2)
    ax.set_ylim(-0.6, 0.6)
    ax.grid(True)
    ax.set_xlabel("x [m]", fontsize=18)
    ax.set_ylabel("y [m]", fontsize=18)
    ax.tick_params(axis='both', labelsize=18)
    plt.tight_layout()

    # 显示图形
    plt.show()
    plt.savefig('exp_eight_vel_on_traj_no_scale.jpg', format='jpg', dpi=300, bbox_inches='tight', pad_inches=0.05)