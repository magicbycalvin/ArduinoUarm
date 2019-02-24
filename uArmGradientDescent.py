import matplotlib.pyplot as plt
import numpy as np

DEBUG = 0

# Set the desired position (x,y) in mm
desiredPosition = (30,30)

#Pause time in seconds for each plot to update
pauseTime = 0.0001
# All measurements are in mm
L1 = 148.4
L2 = 55
L3 = 153
L4a = 159.9
L4b = 52
# Set initial anchored position
MX = 0
MY = 0

# Set the angle step size for gradient descent in radians
d = 0.0001
# Gradient descent constant, larger numbers are faster, smaller are more stable
k = 0.5

# Set initial values for the motor angles in degrees
ThetaL = 180
ThetaR = 0

# Convert all degrees to radians
ThetaL = np.deg2rad(ThetaL)
ThetaR = np.deg2rad(ThetaR)

def fwdKin(ThetaL, ThetaR):

    # Angle between the two motors, this should never be less than 0 (probably 
    # shouldn't be less than 15 degrees or so)
    ThetaM = ThetaL - ThetaR

    # Points A and C are easy to find
#    AX = L2 * np.cos(ThetaR)
#    AY = L2 * np.sin(ThetaR)
    CX = L1 * np.cos(ThetaL)
    CY = L1 * np.sin(ThetaL)
    
    # Now for the harder formulas
    # Law of cosines for the diagonal member L5
    L5 = np.sqrt( L1**2 + L2**2 - (2*L1*L2*np.cos(ThetaM)) )
    # Law of sines to find ThetaC1
    ThetaC1 = np.arcsin( (L2/L5) * np.sin(ThetaM) )
    # Law of cosines to find ThetaC3
    ThetaC3 = np.arccos( (L4b**2+L5**2-L3**2) / (2*L4b*L5) )
    ThetaE = ThetaL + ThetaC1 + ThetaC3
    # Find B from C and trig
#    BX = CX - L4b*np.cos(ThetaE)
#    BY = CY - L4b*np.sin(ThetaE)
    # Finally, find the end effector position from C and trig
    EX = CX + L4a*np.cos(ThetaE)
    EY = CY + L4a*np.sin(ThetaE)
    
    return (EX, EY)

G = tuple(np.absolute(np.subtract(fwdKin(ThetaL, ThetaR) - desiredPosition)))
G = np.linalg.norm(G)





















###############################################################################
# Plotting code
    
# Prepare for the plot loop
plt.close('all')
plt.ion()
plt.figure()
plt.hold(True)

while ():
    # Debugging code
    if (DEBUG):
        print('==============================================================')
        print('==============================================================')
        print('L5: ' + str(L5))
        print('ThetaC1: ' + str(np.rad2deg(ThetaC1)))
        print('ThetaC3: ' + str(np.rad2deg(ThetaC3)))
        print('ThetaE: ' + str(np.rad2deg(ThetaE)))
        print( 'A(' + str(AX) + ',' + str(AY) + ')' )
        print( 'B(' + str(BX) + ',' + str(BY) + ')' )
        print( 'C(' + str(CX) + ',' + str(CY) + ')' )
        print( 'E(' + str(EX) + ',' + str(EY) + ')' )
    
    # Clear the axis before we plot
    plt.cla()
    
    # Now we can plot the movement
    plt.plot([MX, AX], [MY, AY], 'r')
    plt.plot([AX, BX], [AY, BY], 'b')
    plt.plot([MX, CX], [MY, CY], 'g')
    plt.plot([BX, EX], [BY, EY], 'm')
    
    # Set the axis limits
    plt.ylim([-250, 250])
    plt.xlim([-400, 100])
    
    # Pause for a moment so the user can see movement
    plt.pause(pauseTime)