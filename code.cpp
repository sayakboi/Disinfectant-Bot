#include<iostream>
#include<string>

enum etmStates {
    ST, CP0, WT, FN

};
enum etmConditions{
    A, B, C, notA, notB, notC
};

int* readBotId()
{
    //Code to read position and converting it to block coordinates
}

int** readObId(int* botID)
{
    //Code to read position of obstacles and convert it toblock coordinates
}


struct cell{
    int potential;// 0 for explored, -1 for forbidden, and 1 to 64 for unexplored 
};

void startPotentialSet(int *bid)
{
    //setting potential of all the preceding cells to 0
    for(int i=0; i<= (*(bid)) ; i++)
    {
        for(int j=0;j<64;j++)
        {
            plane0[i][j].potential = 0;
        }
    } 
    for(int k=0; k<(*(bid+1)) ; k++ )
    {
        plane0[*bid][k].potential = 0;   
    }   

    //setting equipotential row cells for the row containing the bot
    for(int r=(*(bid+1));r<64;++r)
    {
        plane0[*bid][r].potential = 64;
    }
    //setting rows with decreasing potentials(but equipotential row cells) after the bot's row
    for(int e=((*bid)+1);e<64;++e)
    {
        for(int f=0;f<64; ++f)
        {
            plane0[e][f].potential=64-e;
        }
    }   
}



int *botid; // Bot's cell index
int **obid;// Obstacle cells(assuming 8 ir sensors)
int *waypoint;// Waypoint index 
//Initial State
etmStates currentState = ST;
 cell plane0[64][64]; // Defining base plane with 64x64 tiles
int main()
{
    switch(currentState){
        case ST: botid = readBotId();
                 obid = readObId(botid);
                 startPotentialSet(botid);
                 waypoint = botid;
                 currentState = CP0;

            break;
        case CP0:
            break;
        case WT:
            break;
        case FN: std::cout<<"Covering completing";
            break;
        default: std::cout<<"Invalid ETM State"<<std::flush; 
    }



    return 0;
}