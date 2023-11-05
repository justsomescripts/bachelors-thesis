#!/usr/bin/env python3

# Auslesen des Bauplans erzeugt mit Rviz
# Datei muss erst in eine .txt konvertiert werden, da die Datei sonst nicht lesbar ist

import numpy as np
import os


global rpath
global abspath

rpath = 'src/object_movement/rv6l_pick_and_place/bauplan/construct01.txt'
abspath = os.path.abspath(rpath)


#######################################################
# Funktion zur Positionsauswertung
#######################################################

def get_position():
        
    with open(abspath,'r') as bauplan:
        inhalt = bauplan.readlines()

    ########### Fuer jedes Objekt im Bauplan eine Schleife ###########
    ###### Erste Schleife ######

    for einzelwert in inhalt:                                
        if einzelwert == '* box_g\n':           # Suche nach dem ersten Objekt ueber die Objektbezeichnung aus dem generierten Bauplan
                                                # \n und * muessen mit eingegeben werden           
            i = inhalt.index('* box_g\n')       # auslesen der Position in der Liste
            pos1 = inhalt[i + 1]                # Position ist in der Liste ein Index hoeher, als die Objektgroesse
            len1 = inhalt[i + 5]

            inhalt.remove(einzelwert)           # Objektgroesse und Position werden nach dem auslesen entfernt, damit diese nicht doppelt erkannt werden
            inhalt.remove(pos1)
            inhalt.remove(len1)

            break
        else:
            continue

    pos1_list = pos1.split()                     # pos1 (string) wird in eine Liste transformiert
    len1_list = len1.split()                     # len1 (string) wird in eine Liste transformiert


    ###### zweite Schleife ######

    for einzelwert in inhalt:
        if einzelwert == '* box_r1\n':
              
            i = inhalt.index('* box_r1\n')
            pos2 = inhalt[i + 1]
            len2 = inhalt[i + 5]

            inhalt.remove(einzelwert)                                
            inhalt.remove(pos2)
            inhalt.remove(len2)
            
            break
        else:
            continue

    pos2_list = pos2.split()
    len2_list = len2.split() 


    ###### dritte Schleife ######
    
    for einzelwert in inhalt:
        if einzelwert == '* box_r2\n':

            #groesse = einzelwert
            i = inhalt.index('* box_r2\n')
            pos3 = inhalt[i + 1]
            len3 = inhalt[i + 5]

            inhalt.remove(einzelwert)
            inhalt.remove(pos3)
            inhalt.remove(len3)
            
            break
        else:
            continue

    pos3_list = pos3.split()
    len3_list = len3.split() 


    ###### vierte Schleife ######
    
    for einzelwert in inhalt:
        if einzelwert == '* cyl_b1\n':

            i = inhalt.index('* cyl_b1\n')
            pos4 = inhalt[i + 1]
            len4 = inhalt[i + 5]

            inhalt.remove(einzelwert)
            inhalt.remove(pos4)
            inhalt.remove(len4)
            
            break
        else:
            continue

    pos4_list = pos4.split()
    len4_list = len4.split() 



    ###### fuenfte Schleife ######
    
    for einzelwert in inhalt:
        if einzelwert == '* cyl_b2\n':

            i = inhalt.index('* cyl_b2\n')
            pos5 = inhalt[i + 1]
            len5 = inhalt[i + 5]

            inhalt.remove(einzelwert)
            inhalt.remove(pos5)
            inhalt.remove(len5)
            
            break
        else:
            continue

    pos5_list = pos5.split()
    len5_list = len5.split() 
 

    return pos1_list, pos2_list, pos3_list, pos4_list, pos5_list, len1_list, len2_list, len3_list, len4_list, len5_list
    # [(pos1x pos1y pos1z), (pos2x pos2y pos2z), (pos3x pos3y pos3z), (pos4x pos4y pos4z),(pos5x pos5y pos5z),(dim1x dim1y dim1z), (dim2x dim2y dim2z), (dim3x dim3y dim3z), (dim4x dim4y dim4z), (dim5x dim5y dim5z)]



#######################################################
# Funktion zur Hoehenauswertung
# Roboter soll mit der aktuell niedrigsten hoehe arbeiten
#######################################################

def hoehenauswertung():

    with open(abspath,'r') as bauplan:

        inhalt = bauplan.readlines()

    ps = get_position()
    positionen = list(ps)

    h1 = float(ps[5][2])/2           # box_g
    #print(height1)
    h2 = float(ps[6][2])/2           # box_r1
    #print(height2)
    h3 = float(ps[7][2])/2           # box_r2
    #print(height3)
    h4 = float(ps[8][1])/2           # cyl_b1    [x][1] weil ein Cylinder nur 2 Maße hat
    #print(height4)
    h5 = float(ps[9][1])/2           # cyl_b2    [x][1] weil ein Cylinder nur 2 Maße hat
    #print(height5)


    liste_z = list((positionen[0][2], positionen[1][2], positionen[2][2], positionen[3][2], positionen[4][2])) # z-Werte aus positionen in liste_z gespeichert
    liste = ['','','','','','','','','','']             # Liste mit 10 Elementen erstellen, um die gesuchten Variablen abzulegen
    liste_length = len(liste)

    box_g  = list([0.98, 0.25, 0.91])                   # Feste Ursprungspositionen fuer die Bloecke
    box_r1 = list([1.43, -0.40, 0.92])                  #
    box_r2 = list([1.15, -0.20, 0.92])                  #
    cyl_b1 = list([1.03, -0.49, 0.98])                  #
    cyl_b2 = list([1.41, 0.14, 0.98])                   #
    

    for i in range(liste_length):
            
        index_min_z = np.argmin(liste_z)
        position_min = positionen[index_min_z]           # minimale Position in z-Richtung aus positionen wird in pos_min gespeichert

        ###### \n wird in Variable bsn festgelegt, damit mit pos_min gesucht werden kann
        bsn = '\n'
        pos_min = ' '.join(position_min) + bsn          
        ######
        index_pos_min = inhalt.index(pos_min)
        object_min = inhalt[index_pos_min - 1]          # Objektname des Blockes mit pos_min wird in object_min gespeichert


        ########### Hier werden die benutzten Informationen aus dr Liste geloescht
        inhalt.remove(inhalt[index_pos_min])
        liste_z.remove(liste_z[index_min_z])
        positionen.remove(positionen[index_min_z])

        ########### Hier wird die Information in die jeweilige Liste angefuegt
        liste.append(object_min)
        liste.append(position_min)
        

        if len(liste_z) >= 1:                           # ueberprueft, ob liste_z leer ist und bricht dann ab
            continue
        else:
            break

    del liste[0:10]                                     # loescht ueberfluessige Zeichen am Anfang


    
    for el in liste:                                    # Schleife, um die jeweilige Ursprungsposition des Blocks in ichtiger Reihenfolge in Liste zu speichern
        if el == '* box_r1\n':
            i = liste.index(el)
            liste.insert((i+2), box_r1)
            liste.insert((i+3), h2)

        elif el == '* box_r2\n':
            i = liste.index(el)
            liste.insert((i+2), box_r2)
            liste.insert((i+3), h3)

        elif el == '* box_g\n':
            i = liste.index(el)
            liste.insert((i+2), box_g)
            liste.insert((i+3), h1)

        elif el == '* cyl_b1\n':
            i = liste.index(el)
            liste.insert((i+2), cyl_b1)
            liste.insert((i+3), h4)

        elif el == '* cyl_b2\n':
            i = liste.index(el)
            liste.insert((i+2), cyl_b2)
            liste.insert((i+3), h5)
        else:
            continue

    return liste

    # liste[  [0]     [1]      [2]      [3]     [4]      [5]      [6]      [7]     [8]     [9]     [10]     [11]     
    # liste[[name1] [ziel1] [origin1] [Höhe1] [name2]  [ziel2] [origin2] [Höhe2] [name3] [ziel3] [origin3] [Höhe3]   
    # ...     [12]   [13]     [14]      [15]    [16]    [17]     [18]    [19]   ]
    # ...   [name4] [ziel4] [origin4] [Höhe4] [name5] [ziel5] [origin5] [Höhe5] ]