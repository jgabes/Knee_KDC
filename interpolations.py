import numpy as np
import matplotlib.pyplot as plt

def int_ang_one(th):
    Theta= np.array([0,5,10,15,20,25,30,35,40,45,
                50,55,60,65,70,75,80,85,90])
    phi_top=np.array([32.13,34.72,37.31,39.91,42.51,
                      45.12,47.74,50.37,53.02,55.69,
                      58.38,61.1 ,63.86,66.69,69.6,
                      72.65,75.94,79.64,84.28])
    In_1=np.interp(th, Theta, phi_top)
    plt.plot(th, In_1, '-x')
    return In_1


def int_ang_two(th):
    Theta= np.array([0,5,10,15,20,25,30,35,40,45,50,55,60,65,70,75,80,85,90])
    phi_bottom=np.array([50.79,53.25,54.22,54.28,53.75,52.81,51.57,50.13,48.52,
                         46.79,44.97,43.06,41.09,39.07,37,34.9 ,32.77,30.61,28.43])


    In_2=np.interp(th, Theta, phi_bottom)
    plt.plot(th, In_2, '-x')
    return In_2


def int_mom_arm_one(th):
    Theta= np.array([0,5,10,15,20,25,30,35,40,45,50,55,60,65,70,75,80,85,90])
    top_dl=np.array([1.0699,0.986,0.9133,0.8497,0.7934,0.7432,0.6981,0.6573,
                     0.6202,0.5861,0.5546,0.5254,0.498,0.4722,0.4476,0.4237,
                     0.3998,0.375,0.3466])*.0254+.0254

    In_2=np.interp(th, Theta, top_dl)
    plt.plot(th, In_2, '-x')
    return In_2
    pass


def int_mom_arm_two(th):
    Theta= np.array([0,5,10,15,20,25,30,35,40,45,50,55,60,65,70,75,80,85,90])
    bot_dl=np.array([0.6512,0.6171,0.6044,0.6037,0.6106,0.623,0.64,0.6609,0.6856,
                     0.7139,0.746,0.7823,0.8232,0.8693,0.9214,0.9806,1.0481,1.1258,1.2159])*0.0254+(4.725*.0254)

    In_3=np.interp(th, Theta, bot_dl)
    plt.plot(th, In_3, '-x')
    return In_3







Theta= np.array([0,5,10,15,20,25,30,35,40,45,50,55,60,65,70,75,80,85,90])

int_mom_arm_two(Theta)
