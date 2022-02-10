#!/usr/bin/env python3

# video_ManupulatorControl.py
# Copyright (c) 2019 Dai Owaki <owaki@tohoku.ac.jp>

import matplotlib.pyplot as plt
import numpy as np
import matplotlib as mpl
import matplotlib.gridspec as gridspec
import matplotlib.animation as animation
import math

def InverseKinematics(X, L):

    L1, L2 = L
    x, y = X

    # anlge of the link 1
    th1 = math.atan2(y, x) - math.acos( (L1*L1+(x*x+y*y)-L2*L2)/(2*L1*math.sqrt(x*x+y*y)) )

    # anlge of the link 2
    th2 = math.pi - math.acos( (L1*L1+L2*L2-(x*x+y*y))/(2*L1*L2) )

    return [th1, th2]


def video(x, dt, max_t, params, K, XD):

    m_1, m_2, I_1, I_2, l_g1, l_1, l_g2, g = params
    K_p1, K_p2, K_v1, K_v2= K

    #initial setting of figures
    fig = plt.figure(figsize=(10, 6.))
    gs  = gridspec.GridSpec(2,3)

    # setting for ax1 area
    ax1 = fig.add_subplot(gs[0,0], xlim=(-2, 2), ylim=(-2, 2))
    ax1.set_xlabel('x[m]')
    ax1.set_ylabel('y[m]')
    ax1.grid()

    x_range = np.arange(-2.5, 2.5, 0.1)

    l2,     = plt.plot([], [], 'b-', lw=2, animated = True)                    # draw l2 anime
    l1,     = plt.plot([], [], 'b-', lw=2, animated = True)                    # draw l1 anime
    sholder,= plt.plot(0, 0, 'go', markersize = 10, animated = False)         # draw sholder anime
    hand,   = plt.plot([], [], 'co', markersize = 10, animated = True)          # draw hand
    elbow,  = plt.plot([], [], 'mo', markersize = 10, animated = True)         # draw elbow anime

    hand_traject,  = plt.plot([], [], 'c-', lw=1.5, alpha=1.0, animated = True)   # hand trajectory w anime
    elbow_traject, = plt.plot([], [], 'm-', lw=1.5, alpha=1.0, animated = True)   # elbow trajectory w anime

    for i in range(int(np.size(XD)/2)):
        plt.plot(XD[i][0],XD[i][1],'ro',markersize=11, alpha=0.5)

    # setting for ax2 area
    ax2 = fig.add_subplot(gs[0,1:3], xlim=(0, max_t), ylim=(-1.57, 3.14))
    ax2.set_xlabel('t[s]')
    ax2.set_ylabel('theta[rad]')
    ax2.grid()

    time  =  np.arange(0.0, max_t, dt) # make a list fot time
    line1, = plt.plot(time, x[:,0], 'g-', lw=2, alpha=1.0) #draw th1
    line2, = plt.plot(time, x[:,2], 'b-', lw=2, alpha=1.0) #draw th2

    bar, = plt.plot([], [], 'r-', lw=1, animated=True)           #draw time bar w anime
    p1, = plt.plot([], [], 'go', markersize = 5,  animated=True) #draw th1 point w anime
    p2, = plt.plot([], [], 'bo', markersize = 5,  animated=True) #draw th2 point w anime

    L = [l_1, 2*l_g2]
    Th1 = InverseKinematics(XD[0][:], L)
    plt.plot([0,2.5],[Th1[0],Th1[0]], 'g--', lw=1, alpha=0.5)
    plt.plot([0,2.5],[Th1[1],Th1[1]], 'b--', lw=1, alpha=0.5)

    Th2 = InverseKinematics(XD[1][:], L)
    plt.plot([2.5,5.0],[Th2[0],Th2[0]], 'g--', lw=1, alpha=0.5)
    plt.plot([2.5,5.0],[Th2[1],Th2[1]], 'b--', lw=1, alpha=0.5)

    Th3 = InverseKinematics(XD[2][:], L)
    plt.plot([5.0,7.5],[Th3[0],Th3[0]], 'g--', lw=1, alpha=0.5)
    plt.plot([5.0,7.5],[Th3[1],Th3[1]], 'b--', lw=1, alpha=0.5)

    Th4 = InverseKinematics(XD[3][:], L)
    plt.plot([7.5,max_t],[Th4[0],Th4[0]], 'g--', lw=1, alpha=0.5)
    plt.plot([7.5,max_t],[Th4[1],Th4[1]], 'b--', lw=1, alpha=0.5)

    time_template = 'time = %.2f s'
    time_text = ax1.text(0.54, 0.125, '', transform=ax1.transAxes)

    # setting for ax4 area
    ax4 = fig.add_subplot(gs[1,0])
    ax4.tick_params(labelbottom="off",bottom="off")
    ax4.tick_params(labelleft="off",left="off")
    ax4.set_xticklabels([])

    params_text1 = ax4.text(0.1, 0.93, '', transform=ax4.transAxes)
    params_text2 = ax4.text(0.1, 0.83, '', transform=ax4.transAxes)
    params_text3 = ax4.text(0.1, 0.73, '', transform=ax4.transAxes)
    params_text4 = ax4.text(0.1, 0.63, '', transform=ax4.transAxes)
    params_text5 = ax4.text(0.1, 0.53, '', transform=ax4.transAxes)
    params_text6 = ax4.text(0.1, 0.43, '', transform=ax4.transAxes)
    params_text7 = ax4.text(0.1, 0.33, '', transform=ax4.transAxes)
    params_text8 = ax4.text(0.1, 0.23, '', transform=ax4.transAxes)
    params_text9 = ax4.text(0.1, 0.13, '', transform=ax4.transAxes)
    params_text10 = ax4.text(0.1, 0.03, '', transform=ax4.transAxes)

    plt.tight_layout()

    #initial function for animation
    def init():
        return hand_traject, elbow_traject, l2, l1, sholder, elbow, hand, line1, line2, bar, p1, p2, time_text,\
                params_text1, params_text2, params_text3, params_text4, params_text5, params_text6, params_text7, params_text8, params_text9, params_text10, \

    #function for animation
    def anime(i):
        next_elx   = 0 + l_1*math.cos(x[i,0])  # x position of elbow mass
        next_ely   = 0 + l_1*math.sin(x[i,0])  # y position of elbow mass
        next_hax   = next_elx + 2*l_g2*math.cos(x[i,0]+x[i,2])  # x position of hand mass
        next_hay   = next_ely + 2*l_g2*math.sin(x[i,0]+x[i,2])  # y position of hand mass
        next_l1x   = [0, next_elx]       # x positions for l1
        next_l1y   = [0, next_ely]       # y positions for l1
        next_l2x   = [next_elx, next_hax] # x positions for leg
        next_l2y   = [next_ely, next_hay] # y positions for legs
        next_bx   = [time[i], time[i]]  # x positions for time bar
        next_by   = [-30, 30]           # y positions for time bar

        next_etrx   = l_1*np.cos(x[0:i,0]) # x positions for elbow traject
        next_etry   = l_1*np.sin(x[0:i,0]) # y positions for elbow traject
        elbow_traject.set_data(next_etrx, next_etry)

        next_htrx   = l_1*np.cos(x[0:i,0]) + 2*l_g2*np.cos(x[0:i,0]+x[0:i,2]) # x positions for hand traject
        next_htry   = l_1*np.sin(x[0:i,0]) + 2*l_g2*np.sin(x[0:i,0]+x[0:i,2]) # y positions for hand traject
        hand_traject.set_data(next_htrx, next_htry)


        l1.set_data(next_l1x, next_l1y)          # set l1 positions
        l2.set_data(next_l2x, next_l2y)          # set l2 positions
        elbow.set_data(next_elx, next_ely)       # set elbow position
        hand.set_data(next_hax, next_hay)        # set hand position
        bar.set_data(next_bx, next_by)           # set time bar position
        p1.set_data(time[i], x[i,0])  # set ke position
        p2.set_data(time[i], x[i,2])# set pe position

        time_text.set_text(time_template % time[i]) # display timer

        params_text1.set_text('m1   = %.3f kg' % (m_1)) # display params
        params_text2.set_text('m2   = %.3f kg' % (m_2)) # display params
        params_text3.set_text('I1   = %.3f kg m^2' % (I_1)) # display params
        params_text4.set_text('I2   = %.3f kg m^2' % (I_2)) # display params
        params_text5.set_text('l_g1 = %.3f m' % (l_g1)) # display params
        params_text6.set_text('l_1  = %.3f m' % (l_1)) # display params
        params_text7.set_text('l_g2 = %.3f m' % (l_g2)) # display params
        params_text8.set_text('g    = %.3f m/s^2' % (g)) # display params
        params_text9.set_text('Kp1,Kp2  = %.3f, %.3f' % (K_p1, K_p2)) # display params
        params_text10.set_text('Kv1,Kv2 = %.3f, %.3f' % (K_v1, K_v2)) # display params


        return hand_traject, elbow_traject, l2, l1, sholder, elbow, hand, line1, line2, bar, p1, p2, time_text,\
                params_text1, params_text2, params_text3, params_text4, params_text5, params_text6, params_text7, params_text8, params_text9, params_text10, \


    ani = animation.FuncAnimation(fig, anime, np.arange(1, len(x)), interval=dt*1.0e+3, blit=True, init_func=init)
    #ani.save('ManipulatorControl.mp4', writer='ffmpeg')

    plt.show()
