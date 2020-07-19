#include<iostream>
#include<string>
#include<math.h>
#include<cmath>

enum etmStates {
    ST, CP0, WT, FN

};
enum etmConditions{
    A, B, C, notA, notB, notC
};

int readBotId(int botid[2])
{
    //Code to read position and converting it to block coordinates
}

void readObId(int obid[8][2],int botID[2])
{
    //Code to read position of obstacles and convert it toblock coordinates
}


struct cell{
    int potential;// 0 for explored, -1 for forbidden, and 1 to 64 for unexplored 
};

void assignpoint(int a[2], int b[2])
{
    b[0] = a[0];
    b[1] = a[1];
}

int compare(int a[2],int b[2])
{
    int i=1;
    for(int r=0;r<2;++r)
    {
        if(!(a[r]==b[r]))
        {
            i=0;
        }
    }
    if(i==1)
        return 1;
    else
        return 0;
}


void startPotentialSet(int bid[2], cell plane0[64][64])
{
   
    //setting rows with decreasing potentials(but equipotential row cells) after the bot's row
    for(int e=0;e<64;++e)
    {
        for(int f=0;f<64; ++f)
        {
            plane0[e][f].potential=64-e;
        }
    }   
}

void keepidle()
{
    //Keep the bot idle
}

int movethere(int waypoint[2])
{   float old_err = 0;
    float cur_err = 0; 
    int goal[2] = {0.5*waypoint[0]+0.25,0.5*waypoint[1]+0.25};
    lb:int Botid[2];// Get bot's position
    int pose[2] = {0.5*Botid[0]+0.25,0.5*Botid[1]+0.25};//Each cell is 0.5*0.5 m area
    int error[2] = {goal[0]-pose[0],goal[1]-pose[1]};
    float delta = 0.05;//Acceptable error
    if((sqrt(pow(error[0],2)+pow(error[1],2)))<delta)
    {
        float k=(float)((5*(1-exp(-0.5*(sqrt(pow(error[0],2)+pow(error[1],2)))))))/(float)((sqrt(pow(error[0],2)+pow(error[1],2)))) ;
        float u[2] = {k*(float)(error[0]),k*(float)(error[1])};
        float phi_d = atan2((u[0]),(u[1]));
        float v= sqrt((pow(u[0],2))+(pow(u[1],2)));
        float theta;//Code to get angle of the bot
        cur_err = phi_d - theta;
        float int_err = old_err + cur_err;
        float diff_err = cur_err - old_err;
        float kp = 5;
        float kd = 0.5;
        float ki = 5;
        float w = kp*cur_err+kd*diff_err+ki*int_err;
        float L;//Base length
        float R;//Wheel Radius
        float vr = (2*v+w*L)/(2*R);
        float vl = (2*v-w*L)/(2*R);
        //Code to output this on the bot
        goto lb;


    }
    return 1;
    
}


 void Nearcells(int* botid, int a[8][2])
 {  //make all members of the array as -5
    for(int ac=0;ac<9;++ac)
    {
        for(int ab=0;ab<2;++ab)
        {
            a[ac][ab]=-5;
        }
    }
    int x=*botid,y=*(botid+1);
    if((x+1)<64)
    {
        a[1][0]=x+1;
        a[1][1]=y;
       
        if((y+1)<64)
        {
        a[2][0]=x+1;
        a[2][1]=y+1;
        }
        if((y-1)>-1)
        {
        a[3][0]=x+1;
        a[3][1]=y-1;
        }

    }   
    if((x-1)>-1)
    {
        a[4][0]=x-1;
        a[4][1]=y;
        
        if((y+1)<64)
        {
        a[5][0]=x-1;
        a[5][1]=y-1;
        }
        if((y-1)>-1)
        {
        a[6][0]=x-1;
        a[6][1]=y-1;
        }
    }
    if((y+1)<64)
    {
        a[7][0]=x;
        a[7][1]=y+1;
    }
    if((y-1)>-1)
    {
        a[8][0]=x;
        a[8][1]=y-1;
    }
   

}
void forbiddenAssign(int obid[8][2],cell plane0[64][64])
{
    for(int i=0;i<8;++i)
    {
        plane0[(obid[i][0])][(obid[i][1])].potential=-1; //Here only the obstacle cells are assigned -1 potential
    }
    int nearforbidden[8][2];
    for(int j=0;j<8;++j)
    {
        Nearcells(obid[j],nearforbidden);
        for(int k=0;k<8;++k)
        {
            plane0[(nearforbidden[k][0])][(nearforbidden[k][1])].potential=-1; //Here the cells near to the obstacle cells are assigned -1 potential
        }
    }

} 
int retcost(int a[2],int b[2])
{
    if((a[0]>0)&&(a[1]>0)&&(b[0]>0)&&(b[1]>0))
    {
        int dist = sqrt(pow((b[0]-a[0]),2)+pow((b[1]-a[1]),2));//Absolute distance
        int angle1 =atan2((b[0]-a[0]),(b[1]-a[1]));// Angle between 0 and pi(negative also possible)
        //include code for getting orientation of bot
        int angle;//Get absolute difference between bot and the destination
        int distCost = 5;// Distance cost
        int angleCost = 5;// Turning cost
        int netcost = dist*distCost + angle*angleCost;// Net cost
        return netcost;
    } 
    else
    {
        return -1;
    }
        
}

int eligiblecells(cell plane0[64][64],int a[8][2])
{
    int x=1;
    for(int i=0;i<8;++i)
    {
        if(plane0[a[i][0]][a[i][1]].potential<0)
        {
            x=-1;
        }
    }
    if(x>0)
        return 1;
    else
        return -1;
    
    
}
void updateWaypoint0(int waypoint[2],int botid[2], int nearcells[8][2], cell plane0[64][64])
{
    if(plane0[(botid[1])][(botid[2])].potential>0)
    {
        int upcell[2]={-1,-1}, downcell[2]={-1,-1};
        if(plane0[(botid[0])][(botid[1]+1)].potential>0)
        {
            upcell[0] = botid[0];
            upcell[1] = botid[1]+1;
        }
        if(plane0[(botid[0])][(botid[1]-1)].potential>0)
        {
            downcell[0] = botid[0];
            downcell[1] = botid[1]-1;
        }
        
        int upcost = retcost(botid,upcell);
        int downcost = retcost(botid,downcell);
        if((upcost>=0)&&(downcost>=0))
        {
            if(upcost<=downcost)
            {
                assignpoint(upcell,waypoint);
            }
            else
            {
                
                assignpoint(downcell,waypoint);
            }
        }
        else if((downcost<0)&&(upcost>0))
        {
            assignpoint(upcell,waypoint);
        }
        else if((downcost>0)&&(upcost<0))
        {
            assignpoint(downcell,waypoint);
        }

        




    }
    else
    {
       if(eligiblecells(plane0,nearcells)>0)
       {
           assignpoint(waypoint,nearcells[0]);
           for(int i=1;i<8;++i)
           {
               if((plane0[nearcells[i][0]][nearcells[i][1]].potential)>(plane0[waypoint[0]][waypoint[1]].potential))
                    assignpoint(nearcells[i],waypoint); 
               else
                {
                    if((plane0[nearcells[i][0]][nearcells[i][1]].potential)==(plane0[waypoint[0]][waypoint[1]].potential))
                    {
                        int d1 = retcost(botid,nearcells[i]);
                        int d2 = retcost(botid,waypoint);
                        if(d1<d2)
                        {
                            assignpoint(nearcells[i],waypoint);
                        }
                    }
                }
                
           } 
       } 
       else
       {
          if(plane0[waypoint[0]][waypoint[1]].potential>0);
          else
          {
                  waypoint[0]=-5;
                  waypoint[1]=-5;
              
              
          } 
       }

    }
    

       
}
void task()
{
    // Add code to clean
}



int botid[2]; // Bot's cell index
int obid[8][2];// Obstacle cells(assuming 8 ir sensors)
int waypoint[2];// Waypoint index 
cell plane0[64][64]; // Defining base plane with 64x64 tiles
int main()
{

    //Initial State
    etmStates currentState = ST;
//   while(1) Uncomment when you want to continuously run the etm
//   {
        switch(currentState){
            case ST: readBotId(botid);
                    readObId(obid,botid);
                    startPotentialSet(botid,plane0);
                    waypoint[1] = botid[1];
                    waypoint[2] = botid[2];
                    currentState = CP0;
                    keepidle();
                    currentState=CP0;

                break;
            case CP0: readBotId(botid);
                    readObId(obid,botid);
                    forbiddenAssign(obid,plane0);
                    int nearcells[8][2];
                    Nearcells(botid,nearcells);
                    updateWaypoint0(waypoint,botid,nearcells,plane0);
                    int a[2] = {-5,-5};
                    int reached = 0;
                    if((!(compare(waypoint,botid)))&&(!(compare(waypoint,a))));
                    {
                        movethere(waypoint);
                        currentState = WT;
                    }
                    if(compare(waypoint,botid))
                    {
                        currentState = WT;
                    }
                    if(compare(waypoint,a))
                    {
                        currentState =FN;
                    }

                    
                    
                break;
                 
            case WT: task();
                break;
            case FN: std::cout<<"Covering completing"<<std::flush;
                break;
            default: std::cout<<"Invalid ETM State"<<std::flush; 
        }
//   }




    return 0;
}