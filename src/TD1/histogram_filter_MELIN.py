import numpy as np
import pylab as plt
import time
import random

#retourne une distribution de probabilite uniforme p sur une grille de taille h w
def init_positions(h,w):
    positions = np.zeros((h,w))
    # TODO Question 1
    p = 1.0/(h*w)
    for i in range(h):
        for j in range(w):
            positions[i,j] = p;
    return(positions)


#met a jour les probabilites de positions du robot (positions) suivant la decision de mouvement du robot (motion) et sa chance de reussir le deplacement p_move
def move(positions,motion):
    (h,w) = positions.shape
    new_positions = np.zeros((h,w))
    for i in range(h):
        for j in range(w):
            # TODO Question 2
            new_positions[i,j]=0
            l = i-motion[0]
            k = j-motion[1]
            if (motion[0]==0 and motion[1]==0):
                new_positions[i,j]=positions[(l)%h,(k)%w]
            elif (l >=0 and l<h and k>=0 and k<w):
                new_positions[i,j]=positions[(l)%h,(k)%w]*p_move
            else:
                new_positions[i,j]=positions[(l)%h,(k)%w]*(1-p_move)
    #print new_positions
    return(new_positions)


#met a jour la mesure de probabilite de position du robot (positions) suivant la valeur mesuree par les capteurs (measurement) et la fiabilite du capteur (sensor_right)
def sense(positions,measurement):
    (h,w) = positions.shape
    new_positions = np.zeros((h,w))
    sum=0;
    for i in range(h):
        for j in range(w):
            if (measurement==colors[i][j]):
                new_positions[i,j]=positions[i,j]*sensor_right
            else:
                new_positions[i,j]=positions[i,j]*(1-sensor_right)
            sum+=new_positions[i,j]
    for i in range(h):
        for j in range(w):
            new_positions[i,j]=new_positions[i,j]/sum
    print new_positions

    return(new_positions)

#QUESTION 3 : Retourne la somme des probabilites calculees par sense()
def getSum(positions,measurement):
    (h,w) = positions.shape
    new_positions = np.zeros((h,w))
    sum=0;
    global_sum=0
    for i in range(h):
        for j in range(w):
            if (measurement==colors[i][j]):
                new_positions[i,j]=positions[i,j]*sensor_right
            else:
                new_positions[i,j]=positions[i,j]*(1-sensor_right)
            sum+=new_positions[i,j]

    for i in range(h):
        for j in range(w):
            new_positions[i,j]=new_positions[i,j]/sum
    for i in range(h):
        for j in range(w):
            global_sum+=new_positions[i,j]

    return(global_sum)

# applique le filtre a histogramme pour chacun des mouvements du robot
def filter(positions, motions, measurements):
    plt.ion()
    l=len(motions)
    for i in range(l):
        positions = move(positions,motions[i])
        positions = sense(positions,measurements[i])

        #QUESTION 3 : Verifier que la sommes des probabilites des dalles est bien de 1 (decommenter les lignes suivantes)
        #print("La somme des probabilites des dalles est de :")
        #print getSum(positions,measurements[i])
        #affiche les probabilites de position du robot a la fin de la phase de mesure
        im = plt.imshow(positions,interpolation='nearest')
        plt.pause(1)
    return(positions)

#lance le filtre sur le jeu de donnee fourni, et affiche la
#position du robot estimee par le filtre
def handle_test_case(colors, motions, measurements):
    h = len(colors)
    w = len(colors[0])
    positions = init_positions(h,w) #Toute la grille a une proba de 1/(h*w)
    positions = filter(positions, motions, measurements)     #effectue la mise a jour des probabilites de position du robot
    print positions
    #print getSum(positions,measurements)
    ind = np.argmax(positions)
    x = ind / w
    y = ind - w*x
    print "-- Maximal probability :", positions[x,y]
    print "-- Inferred position   :", x, ",", y


################################################################
# TESTS
################################################################

print "Test case 1"

#couleur de chaque position de la grille
colors = [['red', 'red', 'red', 'red' , 'red','red', 'green', 'red', 'red' , 'red'],
          ['red', 'red', 'red', 'red', 'red','red', 'red', 'green', 'orange' , 'red'],
          ['red', 'red', 'red', 'green', 'red','red', 'red', 'red', 'red', 'red'],
          ['red', 'red', 'red', 'red', 'red','red', 'red', 'red', 'red', 'red']]

#mouvements effectues
motions = [[0,0],[0,1],[1,0],[1,0],[0,1],[-1,0],[1,0],[0,0],[0,1]]

#valeurs observee au cours du mouvement
measurements = ['green', 'red', 'green' ,'green', 'orange','green','green','orange','red']

#probabilite que la mesure du senseur soit fiable
sensor_right = 0.9
#probabilite que le mouvement demande soit effectue
p_move = 0.8

handle_test_case(colors, motions, measurements)

#QUESTION 2 : Pourquoi 2 couleurs ?
#LEs deux couleurs representent differentes probabilites. Les caracteristiques
#de couleur rouge ont en effet plus de chance d'etre occupe par le robot que LEs
#cases de couleurs bleues.

#QUESTION 3 : voir la fonction getSum()      [ligne 57 (Definition)]

#QUESTION 4 :
#sensor_right = 1
#[ nan  nan  nan  nan  nan  nan  nan  nan  nan  nan]
# [ nan  nan  nan  nan  nan  nan  nan  nan  nan  nan]
# [ nan  nan  nan  nan  nan  nan  nan  nan  nan  nan]
# [ nan  nan  nan  nan  nan  nan  nan  nan  nan  nan]]
#-- Maximal probability : nan
#-- Inferred position   : 0 , 0

#On a un capteur qui ne se trompe pas du point de vue du modele (sensor_right=1). Or, on sait du fait de la structure
#des tests que le capteur se trompe. Il detecte donc une case qui n existe pas d ou la sortie.
