#import matplotlib
import matplotlib.pyplot as plt
import math
from math import acos, atan2, sqrt

RF_COXA = 1
RF_FEMUR = 2
RF_TIBIA = 3
X_COXA = 100
Y_COXA = 100

L_COXA = 68
L_FEMUR = 82
L_TIBIA = 166

COXA = 0
FEMUR = 1
TIBIA = 2

class FemurError(Exception):
    pass
class TibiaError(Exception):
    pass

def sq(x):  
    return x*x
# Convert radians to servo position offset.
def radToServo(rads):
    return int(rads*195.56959407132098)

def legIK(X, Y, Z):
    """ Compute leg servo positions. """
    ans = [0,0,0]    # (coxa, femur, tibia)
    #print "legIK", X, Y, Z
    # first, make this a 2DOF problem... by solving coxa
    ans[COXA] = radToServo(atan2(X,Y))
    TrueX = int(sqrt(sq(X)+sq(Y))) - L_COXA
    im = int(sqrt(sq(TrueX)+sq(Z)))  # length of imaginary leg
    
    # get femur angle above horizon...
    q1 = -atan2(Z,TrueX)
    d1 = sq(L_FEMUR)-sq(L_TIBIA)+sq(im)
    d2 = 2*L_FEMUR*im
    
    try:
        q2 = acos(d1/float(d2))
    except:
        #print "femur = d1", d1, "d2", d2, d1/float(d2)
        raise FemurError("Femur error acos(%d/%f) = acos(%f)" % (d1, float(d2), d1/float(d2)))
    ans[FEMUR] = radToServo(q1+q2)  

    # and tibia angle from femur...
    d1 = sq(L_FEMUR)-sq(im)+sq(L_TIBIA)
    d2 = 2*L_TIBIA*L_FEMUR;
    
    try:
        q = acos(d1/float(d2))
    except:
        #print "tibia = d1", d1, "d2", d2, d1/float(d2)
        raise TibiaError("Tibia error acos(%d/%f) = acos(%f)" % (d1, float(d2), d1/float(d2)))
    ans[TIBIA] = radToServo(q-1.57)

    return ans

minValues = [161, 162, 95, 0, 70, 161, 95, 0, 193, 162, 95, 0, 41, 162, 95, 0, 221, 162]
maxValues = [828, 850, 1005, 0, 980, 852, 1005, 0, 861, 864, 1005, 0, 940, 840, 1005, 0, 795, 858]
neutrals = [508, 516, 754, 0, 364, 494, 731, 0, 510, 498, 748, 0, 669, 506, 745, 0, 218, 186]
signs = [False, True, False, False, True, True, False, False, True, True, False, True, False, True, False, True, True, True]

def verify_joint(jointId, angle):
    angle = neutrals[jointId-1] + (angle if signs[jointId-1] else -angle)
    return minValues[jointId-1] <= angle <= maxValues[jointId-1]

def graph_space_at_height(x_min, x_max, y_min, y_max, z, step_size):
    x_solutions = []
    y_solutions = []
    bad_angles = 0
    exceptions = 0
    for x in xrange(x_min, x_max, step_size):
        for y in xrange(y_min, y_max, step_size):
            try:
                coxa, femur, tibia = legIK(x, y, z)
            except (FemurError, TibiaError) as e:
                #raise
                exceptions += 1
                continue
        
            if verify_joint(RF_COXA, coxa) and verify_joint(RF_FEMUR, femur) and verify_joint(RF_TIBIA, tibia):
                x_solutions.append(x)
                y_solutions.append(y)
            else:
                #print "no good angles", coxa, femur,  tibia
                bad_angles += 1

    #print "bad_angles", bad_angles
    #print "exceptions", exceptions
    return x_solutions, y_solutions

x_min, x_max    = -350, 350
y_min, y_max    = -100, 350
z_min, z_max    = 0, 250
step_size       = 10

colors = [
    "#E5000B", "#E21700", "#E13900", "#E05B00", "#DE7D00",
    "#DD9E00", "#DBBE00", "#D4D900", "#B2D800", "#8FD600",
    "#6ED500", "#4CD300", "#2BD200", "#0BD000", "#00CE14",
    "#00CD33", "#00CB53", "#00CA70", "#00C88E", "#00C6AC", 
    "#00C1C5", "#00A1C3", "#0082C2", "#0064C0", "#0045BF",
]

index = 0

from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax3d = fig.gca(projection='3d')


for idx, z in enumerate(xrange(z_min, z_max, 10)):
    xdata, ydata = graph_space_at_height(x_min, x_max, y_min, y_max, z, step_size)
    print "found", len(xdata), "solutions for height", z
    if xdata:
        fig, ax = plt.subplots()
        ax.plot(xdata, ydata, 'o')
        plt.axis([x_min, x_max, y_min, y_max])
        plt.xlabel("front")
        plt.ylabel("sides")
        #plt.zlabel("height")
        ax.set_title('IK solution space at Z=%s' % z)
        plt.savefig("plots/%i_ik_space_at_height_%s" % (index, z))
        index += 1
        #plt.show()
    ax3d.scatter(xdata, ydata, [-z]*len(xdata), c=colors[idx])
#ax3d.legend()
# ax3d.set_xlim3d(x_min, x_max)
# ax3d.set_ylim3d(y_min, y_max)
# ax3d.set_zlim3d(-z_min, -z_max)
# ax3d.set_xlabel('X Label')
# ax3d.set_ylabel('Y Label')
# ax3d.set_zlabel('Z Label')
# plt.show()
#plt.savefig("plots/ik_space_3d")