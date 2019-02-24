"""
Copyright 2016 Calvin Kielas-Jensen

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
"""

import matplotlib.pyplot as plt
import numpy as np

DEBUG = 0

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
# Set max angle in degrees
minAngleL = 105
maxAngleR = 75 

# Set initial values for the motor angles
ThetaL = 180
ThetaR = 0

# Convert all degrees to radians
minAngleL = np.deg2rad(minAngleL)
maxAngleR = np.deg2rad(maxAngleR)
ThetaL = np.deg2rad(ThetaL)
ThetaR = np.deg2rad(ThetaR)

# Prepare for the plot loop
plt.close('all')
plt.ion()
plt.figure()
plt.hold(True)

while (ThetaL > minAngleL and ThetaR < maxAngleR):

    # Angle between the two motors, this should never be less than 0 (probably 
    # shouldn't be less than 15 degrees or so)
    ThetaM = ThetaL - ThetaR
    
    # Points A and C are easy to find
    AX = MX + L2 * np.cos(ThetaR)
    AY = MY + L2 * np.sin(ThetaR)
    CX = MX + L1 * np.cos(ThetaL)
    CY = MY + L1 * np.sin(ThetaL)
    
    # Now for the harder formulas
    # Law of cosines for the diagonal member L5
    L5 = np.sqrt( L1**2 + L2**2 - (2*L1*L2*np.cos(ThetaM)) )
    # Law of sines to find ThetaC1
    ThetaC1 = np.arcsin( (L2/L5) * np.sin(ThetaM) )
    # Law of cosines to find ThetaC3
    ThetaC3 = np.arccos( (L4b**2+L5**2-L3**2) / (2*L4b*L5) )
    ThetaE = ThetaL + ThetaC1 + ThetaC3
    # Find B from C and trig
    BX = CX - L4b*np.cos(ThetaE)
    BY = CY - L4b*np.sin(ThetaE)
    # Finally, find the end effector position from C and trig
    EX = CX + L4a*np.cos(ThetaE)
    EY = CY + L4a*np.sin(ThetaE)
    
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
    
    ThetaL -= 0.01
    ThetaR += 0.01