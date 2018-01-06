from math import *
import pylab as plt
import numpy as np
from matplotlib import animation

# time step
dt = 0.1
# motion uncertainty
R = np.matrix([[1., 0., 0., 0.],
               [0., 1., 0., 0.],
               [0., 0., 1., 0.],
               [0., 0., 0., 1.]])
# initial uncertainty
P =  np.matrix([[5.,0.,0.,0.],
             [0.,5.,0.,0.],
             [0.,0.,100.,0.],
             [0.,0.,0.,100.]])
# next state function // fonction de transition
#QUESTION 1 : Initialisez les matrices de la fonction de transition F et de la fonction de mesure H en fonction des
#donnees de l enonce.Que pouvez vous deduire de la matrice P, representant l incertitude au debut du test,
#sur notre connaissance des conditions initiales ?
F=  np.matrix([[1.,0.,dt,0.],
             [0.,1.,0.,dt],
             [0.,0.,1.,0.],
             [0.,0.,0.,1.]])
 # measurement function // fonction de mesure
H =  np.matrix([[1.,0.,0.,0.],
             [0.,1.,0.,0.]])
# measurement uncertainty
Q =  np.matrix([[1.,0.],
             [0.,1.]])
# identity matrix
I =  np.eye(P.shape[0])

# display settings
scale =0.2
xmax = 75
ymax = 75

#compute approximate values of a gaussian distribution
def bin_matrix(x,P):
    M = np.zeros((xmax,ymax))
    sigma = plt.det(P)
    d_pos = np.zeros((P.shape[0],1))
    P_inv = np.linalg.inv(P)
    sum1=0
    for i in range(xmax):
        for j in range(ymax):
            d_pos[0,0] = x[0]-(i*scale)
            d_pos[1,0] = x[1]-(j*scale)
            exp_value = -0.5*(d_pos.transpose()*P_inv*d_pos)
            M[i,j] = 1./(np.sqrt(2 * np.pi*sigma)) * np.exp( exp_value)
            sum1=sum1+ M[i,j]
    for i in range(xmax):
        for j in range(ymax):
            M[i,j]= M[i,j]/sum1
    return M


# compute position (new_x) and uncertainty (new_P) after movement
def move(x, P):
    new_x = x
    new_P = P

    # TODO : QUESTION 2 : Completer la fonction move(x, P ), qui met a jour les caracteristiques du robot
    #(position et vitesse), ainsi que la matrice de covariance P, en fonction de la position, de la vitesse
    #et de la covariance precedentes, ainsi qu en fonction de la commande de deplacement u. Lancer le premier test.
    #Vous devriez voir se deplacer la distribution representant la position du robot.
    #Comment evolue la precision du filtre ?

    # On voit se deplacer la distribution representant la position du robot.
    # L'ecart-type de la loi normale augmente (elargissement du cercle de distribution)
    # On peut donc en deduire que la precision du robot diminue puisque les valeurs sont
    # reparties de maniere moins homogene autour de la moyenne.
    new_x = F*new_x
    new_P = F*new_P*np.transpose(F) + R

    return (new_x,new_P)

def sense(x,P,Z):
    new_x =x
    new_P=P
    # TODO : QUESTION 3
    Y = Z - (H*x)
    S = H*P*np.transpose(H) + Q
    K = P*np.transpose(H)*np.linalg.inv(S)

    new_x = x + (K*Y)
    new_P = (np.eye(4,4) - (K*H))*P
    return (new_x,new_P)

def filter(x, P, measurements):
    plt.ion()
    l=len(measurements)
    p = bin_matrix(x,P)
    im = plt.imshow(p)
    plt.pause(1)
    for n in range(l):
        # prediction
        (x,P) = move(x,P)
        #plot probabilities after movement
        p = bin_matrix(x,P)
        im = plt.imshow(p)
        plt.pause(0.1)
        # measurement update
        Z = np.matrix([measurements[n]]).transpose()
        (x,P) = sense(x,P,Z)
        #plot probabilities after sensing
        p = bin_matrix(x,P)
        im = plt.imshow(p)
        plt.pause(0.1)
    plt.ioff()
    return [x,P]

def handle_test_case(measurements,initial_xy,final_position):
    x = np.matrix([[initial_xy[0]], [initial_xy[1]], [0.], [0.]])
    res = filter(x, P,measurements)
    filtered_position   = np.array(res[0]).T[0]
    filtered_covariance = np.array(res[1])
    print "-- Estimated position   :", filtered_position
    print "-- Real position        :", final_position
    print "-- Position error       :", np.linalg.norm(final_position-filtered_position)
    print "-- Estimated covariance :", np.linalg.norm(filtered_covariance)

def handle_test_case(measurements,initial_xy,final_position):
    x = np.matrix([[initial_xy[0]], [initial_xy[1]], [0.], [0.]])
    res = filter(x, P,measurements)
    filtered_position   = np.array(res[0]).T[0]
    filtered_covariance = np.array(res[1])
    print "-- Estimated position   :", filtered_position
    print "-- Real position        :", final_position
    print "-- Position error       :", np.linalg.norm(final_position-
                                               filtered_position)
    print "-- Estimated covariance :", np.linalg.norm(filtered_covariance)

################################################################
# Test cases
################################################################
print "Test case 1 (4-dimensional example, error-free measurements)"
#u : [x,y,vx,vy]
u = np.matrix([[0], [0], [0.], [0.]])
measurements = [[5., 10.], [6., 8.], [7., 6.], [8., 4.], [9., 2.], [10., 0.]]
initial_xy = [4., 12.]
final_position = np.array([10.,0.,10.,-20.])
handle_test_case(measurements,initial_xy,final_position)


################################################################
print "Test case 2 (4-dimensional example, errors on measurements and initial conditions)"
#u : [x,y,vx,vy]
u = np.matrix([[0], [0], [0.], [0.]])
measurements = [[5.5, 10.], [6.5, 8.3], [7., 5.4], [6.5, 4.2], [9.3, 1.5], [10, 0.5]]
initial_xy = [4., 11.]
final_position = np.array([10.,0.,10.,-20.])
handle_test_case(measurements,initial_xy,final_position)


##############################################################
#print "Test case 3 (4-dimensional example, errors and robot acceleration)"
#u : [x,y,vx,vy]
#u = np.matrix([[0], [0], [0.], [0.]])
#measurements = [[5., 11.], [4., 12.5], [4., 13.5], [5., 14.], [7., 14.], [10., 13.5]]
#initial_xy = [7., 9.]
#final_position = np.array([10,13.5,40.,-10.])

#handle_test_case(measurements,initial_xy,final_position)



# QUESTION 3 : Corrigez la fonction sense en implementant l algorithme fourni dans le cours. Lancez les tests 1 et 2,
#vous devez voir la precision augmenter lors des premiers deplacements.
#Comparez les erreurs de position obtenues dans les tests avec et sans erreur de mesure, que pouvez-
#vous en deduire sur l efficacite du filtre ?

#On lance les tests 1 et 2 et on obtient les resultats suivants :
#Test case 1 (4-dimensional example, error-free measurements)
#-- Position error       : 4.122347013
## Test case 2 (4-dimensional example, errors on measurements and initial conditions)
#-- Position error       : 5.17399239095
#On remarque que l'erreur en position augmente d'une unite en cas d erreur de mesure.
#Le filtre est plutot efficace puisqu on abouti a une difference de 1 unite entre le cas sans erreur et le cas avec erreur.
 
# QUESTION 4 et 5 : non comprises
