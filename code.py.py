def startPotentialSet(plane0) :
    #setting rows with decreasing potentials(but equipotential row cells) after the bot's row
    for e in range(64):
        for f in range(64):
            plane0[e][f] = 64 - e
    return plane0

def movethere(waypoint) :
    old_err = 0.0
    cur_err = 0.0
    goal = [0.5*waypoint[0]+0.25, 0.5*waypoint[1]+0.25]

    Botid = [0, 0] # Get bot's position
    pose = [0.5*Botid[0]+0.25, 0.5*Botid[1]+0.25] #Each cell is 0.5*0.5 m area
    error = [goal[0]-pose[0], goal[1]-pose[1]]
    delta = 0.05 #Acceptable error

    while sqrt(error[0]**2+error[1]**2)<delta :
        k = ((5*(1-np.exp(-0.5*(np.sqrt(error[0]**2+error[1]**2))))))/((np.sqrt(error[0]**2+error[1]**2)))
        u = [k*error[0], k*error[1]]
        phi_d = np.atan2((u[0]),(u[1]))
        v = np.sqrt(u[0]**2+u[1]**2)
        theta = 0.0 #Code to get angle of the bot
        cur_err = phi_d - theta
        int_err = old_err + cur_err
        diff_err = cur_err - old_err
        kp = 5.0
        kd = 0.5
        ki = 5.0
        w = kp*cur_err+kd*diff_err+ki*int_err
        L = 0.0 #Base length
        R = 0.0 #Wheel Radius
        vr = (2*v+w*L)/(2*R)
        vl = (2*v-w*L)/(2*R)

        #Code to output this on the bot

        Botid = [0, 0] # Get bot's position
        pose = [0.5*Botid[0]+0.25, 0.5*Botid[1]+0.25] #Each cell is 0.5*0.5 m area
        error = [goal[0]-pose[0], goal[1]-pose[1]]
        delta = 0.05 #Acceptable error
    return 1


def Nearcells(botid) :
    #make all members of the array as -5
    a = [ [-5, -5], [-5, -5], [-5, -5], [-5, -5], [-5, -5], [-5, -5], [-5, -5], [-5, -5] ]
    x = botid[0]
    y = botid[1]

    if x+1<64 :
        a[1][0]=x+1
        a[1][1]=y
        if y+1<64 :
            a[2][0]=x+1
            a[2][1]=y+1
        if y-1>-1 :
            a[3][0]=x+1
            a[3][1]=y-1
    if x-1>-1 :
        a[4][0]=x-1
        a[4][1]=y
        if y+1<64 :
            a[5][0]=x-1
            a[5][1]=y-1
        if y-1>-1 :
            a[6][0]=x-1
            a[6][1]=y-1
    if y+1<64 :
        a[7][0]=x
        a[7][1]=y+1
    if y-1>-1 :
        a[8][0]=x
        a[8][1]=y-1

    return a

def forbiddenAssign(obid, plane0) :
    for i in range(8) :
        plane0[(obid[i][0])][(obid[i][1])] = -1 #Here only the obstacle cells are assigned -1 potential
    for j in range(8) :
        nearforbidden = Nearcells(obid[j]);
        for k in range(8) :
            plane0[(nearforbidden[k][0])][(nearforbidden[k][1])] = -1 #Here the cells near to the obstacle cells are assigned -1 potential
    return plane0

def retcost(a, b) :
    if a[0]>0 && a[1]>0 && b[0]>0 && b[1]>0 :
        dist = sqrt((b[0]-a[0])**2+(b[1]-a[1])**2) #Absolute distance
        angle1 = atan2((b[0]-a[0]), (b[1]-a[1])) # Angle between 0 and pi(negative also possible)
        #include code for getting orientation of bot
        angle = 0 #Get absolute difference between bot and the destination
        distCost = 5 # Distance cost
        angleCost = 5 # Turning cost
        netcost = dist*distCost + angle*angleCost # Net cost
        return netcost
    else:
        return -1

def eligiblecells(plane0, a) :
    x = 1
    for i in range(8):
        if plane0[a[i][0]][a[i][1]]<0 :
            x = -1
    if x>0 :
        return 1
    else:
        return -1

def updateWaypoint0(waypoint, botid, nearcells, plane0) :
    if plane0[(botid[1])][(botid[2])]>0 :
        upcell=[-1,-1], downcell=[-1,-1]
        if plane0[(botid[0])][(botid[1]+1)]>0 :
            upcell[0] = botid[0]
            upcell[1] = botid[1]+1
        if plane0[(botid[0])][(botid[1]-1)]>0 :
            downcell[0] = botid[0]
            downcell[1] = botid[1]-1

        upcost = retcost(botid, upcell)
        downcost = retcost(botid, downcell)

        if upcost>=0 && downcost>=0 :
            if upcost<=downcost :
                waypoint = upcell
            else:
                waypoint = downcell
        elif downcost<0 && upcost>0 :
            waypoint = upcell
        elif downcost>0 && upcost<0 :
            waypoint = downcell

    else:
        if eligiblecells(plane0,nearcells)>0 :
            nearcells[0] = waypoint
            for(int i=1;i<8;++i)
            for i in range(1, 8):
                if plane0[nearcells[i][0]][nearcells[i][1]]>plane0[waypoint[0]][waypoint[1]] :
                    waypoint = nearcells[i]
                else:
                    if plane0[nearcells[i][0]][nearcells[i][1]]==plane0[waypoint[0]][waypoint[1]] :
                        int d1 = retcost(botid, nearcells[i])
                        int d2 = retcost(botid, waypoint)
                        if d1<d2 :
                            waypoint = nearcells[i]
        else:
            if plane0[waypoint[0]][waypoint[1]]<=0 :
                waypoint[0] = -5
                waypoint[1] = -5
    return waypoint, botid, nearcells, plane0

def task() :
    # Add code to clean

def keepidle() :
    #Keep the bot idle

def compare(a, b) :
    return a == b

import numpy as np

def main() :
    #Initial State
    currentState = 0

    plane0 = [ [0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0], \
    [0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0], \
    [0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0], \
    [0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0] ]

#   while(1) Uncomment when you want to continuously run the etm
    botid =  readBotId()
    obid = readObId()
    waypoint = botid

    if currentState == 0 :
        plane0 = startPotentialSet(plane0)
        keepidle()
        currentState = 1
    elif currentState == 1:
        plane0 = forbiddenAssign(obid, plane0)
        nearcells = Nearcells(botid)
        waypoint, botid, nearcells, plane0 = updateWaypoint0(waypoint, botid, nearcells, plane0)
        a = [-5,-5]
        reached = 0

        if waypoint!=botid && waypoint!=a :
            movethere(waypoint)
            currentState = 2
        if waypoint==botid :
            currentState = 2
        if waypoint==a :
            currentState = 3
    elif currentState == 2 :
        task();
    elif currentState == 3 :
        print("Covering completing\n")
    else:
        print("Invalid ETM State\n")

if __name__ == '__main__':
    main()
