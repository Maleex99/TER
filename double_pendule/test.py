import pybullet as p
import time
import pybullet_data
from math import pi
from numpy import cos, sin, sqrt, array, absolute, argmin, arctan2

def getMGD(q1, q2):
    y = -sin(q1) * cos(q2) - cos(q1) * sin(q2) - sin(q1)
    z =  cos(q1) * cos(q2) - sin(q1) * sin(q2) + cos(q1)
    return (y, z)

def getMGI(y, z, rx = 0, q1Actuel = 0):
    T23 = -sin(rx)
    T33 = cos(rx)
    E = array([-1, 1])
    c1 = (E * T33 * sqrt(T33**2 + T23**2)) / (T33**2 + T23**2)
    s1 = (E * T23 * sqrt(T23**2 + T33**2)) / (T33**2 + T23**2)
    q1 = arctan2(s1, c1)
    e = absolute(q1 - q1Actuel)
    q1Index = argmin(e)
    q1 = q1[q1Index]
    c2 = -sin(q1) * y + cos(q1) * z - 1
    s2 = -cos(q1) * y + sin(q1) * z
    q2 = arctan2(s2, c2)
    return (q1, q2)




if __name__ == '__main__':
    physicsClient = p.connect(p.GUI)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally

    cubeStartPos = [0, 0, 0]
    cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
    pendule = p.loadURDF("double_pendule1.urdf", cubeStartPos, cubeStartOrientation)

    # Repére O1
    p.addUserDebugLine([0, 0, 0], [0, 1, 0], [0, 255, 0], parentObjectUniqueId = pendule, parentLinkIndex = 0)
    p.addUserDebugLine([0, 0, 0], [0, 0, 1], [0, 0, 255], parentObjectUniqueId = pendule, parentLinkIndex = 0)

    # Repére O2
    p.addUserDebugLine([0, 0, 0], [0, 1, 0], [0, 255, 0], parentObjectUniqueId = pendule, parentLinkIndex = 1)
    p.addUserDebugLine([0, 0, 0], [0, 0, 1], [0, 0, 255], parentObjectUniqueId = pendule, parentLinkIndex = 1)

    # Afficheurs
    posOutilReelDisplay = p.addUserDebugText("[0, 4]", [0, 0, 0.3], [255, 0, 0], parentObjectUniqueId = pendule, parentLinkIndex = 2)
    q1ReelDisplay = p.addUserDebugText("q1 = 0", [0, 0, 0.2], [255, 0, 0], parentObjectUniqueId = pendule, parentLinkIndex = 0)
    q2ReelDisplay = p.addUserDebugText("q2 = 0", [0, 0, 0.2], [255, 0, 0], parentObjectUniqueId = pendule, parentLinkIndex = 1)

    # Parametres
    q1ParamSlider = p.addUserDebugParameter("q1",-pi,pi,0)
    q2ParamSlider = p.addUserDebugParameter("q2",-pi,pi,0)

    # Bouton MGD
    btYZ = p.addUserDebugParameter("Calculer MGD YZ",1,0,0)
    btYZvalue = 0

    # Interface MGI
    yMGI = p.addUserDebugParameter("y",-2,2,0)
    zMGI = p.addUserDebugParameter("z",-2,2,2)
    rxMGI = p.addUserDebugParameter("rx",-pi,pi,0)
    btGoTo = p.addUserDebugParameter("Go to",1,0,0)
    btGoTovalue = 0

    # Camera
    p.resetDebugVisualizerCamera(5,90,-10,[0,0,0])

    while(1):
        p.stepSimulation()
        time.sleep(1. / 240.)

        # Recuperation des valeurs des sliders
        q1Param = p.readUserDebugParameter(q1ParamSlider)
        q2Param = p.readUserDebugParameter(q2ParamSlider)

        # Rotation des rotoïdes
        #p.setJointMotorControl2(pendule,0,p.POSITION_CONTROL,targetPosition=q1Param)
        #p.setJointMotorControl2(pendule,1,p.POSITION_CONTROL,targetPosition=q2Param)

        # Recuperation des valeurs réels des qi
        q1 = p.getJointState(pendule, 0)[0]
        q2 = p.getJointState(pendule, 1)[0]

        # Afficher qi réel
        q1ReelDisplay = p.addUserDebugText("q1 = "+str(round(q1,3)), [0, 0, 0.2], [255, 0, 0], parentObjectUniqueId = pendule, parentLinkIndex = 0, replaceItemUniqueId = q1ReelDisplay)
        q2ReelDisplay = p.addUserDebugText("q2 = "+str(round(q2,3)), [0, 0, 0.2], [255, 0, 0], parentObjectUniqueId = pendule, parentLinkIndex = 1, replaceItemUniqueId = q2ReelDisplay)

        # Récupération position outil
        posOutilReel = p.getLinkState(pendule,2)[0]

        # Affichage position outil
        posOutilReelDisplay = p.addUserDebugText("[" + str(round(posOutilReel[1], 2)) +", "+ str(round(posOutilReel[2], 2)) + "]", [0, 0, 0.2], [255, 0, 0], parentObjectUniqueId = pendule, parentLinkIndex = 2, replaceItemUniqueId = posOutilReelDisplay)

        # CheckBouton MGD
        if p.readUserDebugParameter(btYZ) > btYZvalue:
            btYZvalue = p.readUserDebugParameter(btYZ)

            q1 = p.getJointState(pendule, 0)[0]
            q2 = p.getJointState(pendule, 1)[0]
            MGD = getMGD(q1, q2)
            print([round(MGD[0], 2), round(MGD[1], 2)])

        # CheckBouton Go to        
        if p.readUserDebugParameter(btGoTo) > btGoTovalue:
            btGoTovalue = p.readUserDebugParameter(btGoTo)

            y = p.readUserDebugParameter(yMGI)
            z = p.readUserDebugParameter(zMGI)
            rx = p.readUserDebugParameter(rxMGI)
            q1 = p.getJointState(pendule, 0)[0]
            (q1, q2) = getMGI(y, z, rx, q1)

            p.setJointMotorControl2(pendule,0,p.POSITION_CONTROL,targetPosition=q1)
            p.setJointMotorControl2(pendule,1,p.POSITION_CONTROL,targetPosition=q2)


    p.disconnect()