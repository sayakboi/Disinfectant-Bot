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

void startPotentialSet(int bid[2], cell plane0[64][64])
{
    //setting potential of all the preceding cells to 0
    for(int i=0; i< bid[1] ; i++)
    {
        for(int j=0;j<64;j++)
        {
            plane0[i][j].potential = 0;
        }
    } 
    for(int k=0; k<bid[2] ; k++ )
    {
        plane0[bid[1]][k].potential = 0;   
    }   

    //setting equipotential row cells for the row containing the bot
    for(int r=bid[2];r<64;++r)
    {
        plane0[bid[1]][r].potential = 64;
    }
    //setting rows with decreasing potentials(but equipotential row cells) after the bot's row
    for(int e=bid[1]+1;e<64;++e)
    {
        for(int f=0;f<64; ++f)
        {
            plane0[e][f].potential=64-(e-bid[1]);
        }
    }   
}

void keepidle()
{
    //Keep the bot idle
}

void forbiddenAssign(int obid[8][2],cell plane0[64][64])
{
    for(int i=0;i<8;++i)
    {
        plane0[(obid[i][1])][(obid[i][2])].potential=-1; //Here only the obstacle cells are assigned -1 potential
    }
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
        a[1][1]=x+1;
        a[1][2]=y;
       
        if((y+1)<64)
        {
        a[2][1]=x+1;
        a[2][2]=y+1;
        }
        if((y-1)>-1)
        {
        a[3][1]=x+1;
        a[3][2]=y-1;
        }

    }   
    if((x-1)>-1)
    {
        a[4][1]=x-1;
        a[4][2]=y;
        
        if((y+1)<64)
        {
        a[5][1]=x-1;
        a[5][2]=y-1;
        }
        if((y-1)>-1)
        {
        a[6][1]=x-1;
        a[6][2]=y-1;
        }
    }
    if((y+1)<64)
    {
        a[7][1]=x;
        a[7][2]=y+1;
    }
    if((y-1)>-1)
    {
        a[8][1]=x;
        a[8][2]=y-1;
    }
   

} 
int retcost(int a[2],int b[2])
{
    int dist = sqrt(pow((b[1]-a[1]),2)+pow((b[2]-a[2]),2));//Absolute distance
    int angle = abs(atan2((b[1]-a[1]),(b[2]-a[2])));// Angle between 0 and pi
    int distCost = 5;// Distance cost
    int angleCost = 5;// Turning cost
    int netcost = dist*distCost + angle*angleCost;// Net cost
    return netcost;
}
void updateWaypoint(int waypoint[2],int botid[2], int nearcells[8][2], cell plane0[64][64])
{
    if(plane0[(botid[1])][(botid[2])].potential>0)
    {
       //Not so sure about this part 
    }
    else
    {
       //To be continued 
    }
    

       
}



int botid[2]; // Bot's cell index
int obid[8][2];// Obstacle cells(assuming 8 ir sensors)
int waypoint[2];// Waypoint index 
cell plane0[64][64]; // Defining base plane with 64x64 tiles
int main()
{

    //Initial State
    etmStates currentState = ST;
   
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
                updateWaypoint(waypoint,botid,nearcells,plane0);
                 
            break;
        case WT:
            break;
        case FN: std::cout<<"Covering completing"<<std::flush;
            break;
        default: std::cout<<"Invalid ETM State"<<std::flush; 
    }




    return 0;
}