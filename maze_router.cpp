#include <iostream>
#include <queue>
#include <fstream>
#include <bitset>
using namespace std;

short int*** grid;
short int xlbound = 0;
short int xubound;
short int ylbound = 0;
short int yubound;
unsigned int** place;
short int X, Y, bend, via;

unsigned int route(unsigned short int ls, unsigned short int xs, unsigned short int ys, unsigned short int lt, unsigned short int xt, unsigned short int yt){
    cout << "*************** Begin ***************\n";
    int data, top;
    unsigned int counter;
    bool target = false;
    bool source = false;
    int pcwmask = 4194303;          //mask for PathCount write
    int xwmask = 4290781183;        //mask for x coordinate write
    int ywmask = 4294959119;        //mask for y coordinate write
    int predwmask = 4294967281;     //mask for predecessor write
    int layerwmask = 4294967294;    //mask for layer write
    int pcrmask = 4290772992;       //mask for PathCount read
    int xrmask = 4186112;           //mask for x coordinate read
    int yrmask = 8176;              //mask for y coordinate read
    int predrmask = 14;             //mask for predecessor read
    int layerrmask = 1;             //mask for layer read
    int pcshift = 22;               //PC position
    int xshift = 13;                //X coordinate position
    int yshift = 4;                 //Y coordinate position
    int predshift = 1;              //Predecessor position
    int xcoor, ycoor, pc, pred, layer;
    int pc_n, xcoor_n, ycoor_n, pred_n, layer_n;
    priority_queue <int, vector<int>, greater<int> > wf;        //create heap for storing Wave Fronts - smallest PC on top
    cout << "Heap Created\n";
    //32 bit data stored in WF:
    //bit 0: layer (0=layer 1, 1=layer 2)
    //bits 1-3: predecessor bits (N=1, E=2, S=3, W=4, U=5, D=6, 0 for source
    //bits 4-12: y coordinate
    //bits 13-21: x coordinate
    //bits 22-31: Path Cost
    //store source data in heap to begin expansion
    data = 0;
    data = (data & pcwmask) | (1 << pcshift);       //write pathcost pc
    data = (data & xwmask) | (xs << xshift);        //write x
    data = (data & ywmask) | (ys << yshift);        //write y
    data = (data & predwmask) | (0 << predshift);   //write pred
    data = (data & layerwmask) | ls;
    wf.push(data);
    while(!target){     //expand till target not reached
        cout << "Begin Expansion\n";
        if(wf.empty()) break;   //if heap empty, quit
        top = wf.top();         //store the top element of WF for expansion
        grid[xs][ys][ls] = grid[xs][ys][ls] | (1 << 15);    //mark source cell as reached
        wf.pop();           //delete top element from WF
        pc = (top & pcrmask) >> 22;     //Read corresponding bits from top element
        xcoor = (top & xrmask) >> 13;
        ycoor = (top & yrmask) >> 4;
        pred = (top & predrmask) >> 1;
        layer = (top & layerrmask);
        //cout << "WF Cell xcoor: " << xcoor << " ycoor: " << ycoor << " Layer: " << layer << endl;

        //expand cell
        if(xcoor-1 >= xlbound){ //within x bound
            if(!(grid[xcoor-1][ycoor][layer] >> 15)){  //if cell not reached
                grid[xcoor-1][ycoor][layer] = grid[xcoor-1][ycoor][layer] | (1 << 15);    //mark cell as reached
                pc_n = (grid[xcoor-1][ycoor][layer] & 4095);   //cell cost
                if(pc_n != 0){     //proceed only if the cell is not an obstacle
                    cout << "West Cell\n";
                    pred_n = 2;     //reached from east
                    if(pred == pred_n || pred == 0) pc_n = pc + pc_n;  //old pc + cell cost
                    else pc_n = pc + pc_n + bend;  //old pc + cell cost + bend cost
                    xcoor_n = xcoor - 1;
                    ycoor_n = ycoor;
                    layer_n = layer;
                    //write into WF
                    data = 0;
                    data = (data & pcwmask) | (pc_n << pcshift);        //write pathcost pc
                    data = (data & xwmask) | (xcoor_n << xshift);       //write x
                    data = (data & ywmask) | (ycoor_n << yshift);       //write y
                    data = (data & predwmask) | (pred_n << predshift);  //write pred
                    data = (data & layerwmask) | layer_n;               //write layer
                    wf.push(data);
                    grid[xcoor-1][ycoor][layer] = grid[xcoor-1][ycoor][layer] | (pred_n << 12);   //store pred in grid
                    if(xcoor_n == xt && ycoor_n == yt && layer_n == lt) target = true;   //check if target reached
                }
            }
        }
        if(xcoor+1 < xubound){ //within x bound
            if(!(grid[xcoor+1][ycoor][layer] >> 15)){  //if cell not reached
                grid[xcoor+1][ycoor][layer] = grid[xcoor+1][ycoor][layer] | (1 << 15);    //mark cell as reached
                pc_n = (grid[xcoor+1][ycoor][layer] & 4095);   //cell cost
                if(pc_n != 0){     //proceed only if the cell is not an obstacle
                    cout << "East Cell\n";
                    pred_n = 4;     //reached from west
                    if(pred == pred_n || pred == 0) pc_n = pc + pc_n;  //old pc + cell cost
                    else pc_n = pc + pc_n + bend;  //old pc + cell cost + bend cost
                    xcoor_n = xcoor + 1;
                    ycoor_n = ycoor;
                    layer_n = layer;
                    //write into WF
                    data = 0;
                    data = (data & pcwmask) | (pc_n << pcshift);        //write pathcost pc
                    data = (data & xwmask) | (xcoor_n << xshift);       //write x
                    data = (data & ywmask) | (ycoor_n << yshift);       //write y
                    data = (data & predwmask) | (pred_n << predshift);  //write pred
                    data = (data & layerwmask) | layer_n;               //write layer
                    wf.push(data);
                    grid[xcoor+1][ycoor][layer] = grid[xcoor+1][ycoor][layer] | (pred_n << 12);   //store pred in grid
                    if(xcoor_n == xt && ycoor_n == yt && layer_n == lt) target = true;   //check if target reached
                }
            }
        }
        if(ycoor-1 >= ylbound){ //within y bound
            if(!(grid[xcoor][ycoor-1][layer] >> 15)){  //if cell not reached
                grid[xcoor][ycoor-1][layer] = grid[xcoor][ycoor-1][layer] | (1 << 15);    //mark cell as reached
                pc_n = (grid[xcoor][ycoor-1][layer] & 4095);   //cell cost
                if(pc_n != 0){     //proceed only if the cell is not an obstacle
                    cout << "South Cell\n";
                    pred_n = 1;     //reached from north
                    if(pred == pred_n || pred == 0) pc_n = pc + pc_n;  //old pc + cell cost
                    else pc_n = pc + pc_n + bend;  //old pc + cell cost
                    xcoor_n = xcoor;
                    ycoor_n = ycoor - 1;
                    layer_n = layer;
                    //write into WF
                    data = 0;
                    data = (data & pcwmask) | (pc_n << pcshift);        //write pathcost pc
                    data = (data & xwmask) | (xcoor_n << xshift);       //write x
                    data = (data & ywmask) | (ycoor_n << yshift);       //write y
                    data = (data & predwmask) | (pred_n << predshift);  //write pred
                    data = (data & layerwmask) | layer_n;               //write layer
                    wf.push(data);
                    grid[xcoor][ycoor-1][layer] = grid[xcoor][ycoor-1][layer] | (pred_n << 12);   //store pred in grid
                    if(xcoor_n == xt && ycoor_n == yt && layer_n == lt) target = true;   //check if target reached
                }
            }
        }
        if(ycoor+1 < yubound){ //within y bound
            if(!(grid[xcoor][ycoor+1][layer] >> 15)){  //if cell not reached
                grid[xcoor][ycoor+1][layer] = grid[xcoor][ycoor+1][layer] | (1 << 15);    //mark cell as reached
                pc_n = (grid[xcoor][ycoor+1][layer] & 4095);   //cell cost
                if(pc_n != 0){     //proceed only if the cell is not an obstacle
                    cout << "North Cell\n";
                    pred_n = 3;     //reached from south
                    if(pred == pred_n || pred == 0) pc_n = pc + pc_n;  //old pc + cell cost
                    else pc_n = pc + pc_n + bend;  //old pc + cell cost + bend cost
                    xcoor_n = xcoor;
                    ycoor_n = ycoor + 1;
                    layer_n = layer;
                    //write into WF
                    data = 0;
                    data = (data & pcwmask) | (pc_n << pcshift);        //write pathcost pc
                    data = (data & xwmask) | (xcoor_n << xshift);       //write x
                    data = (data & ywmask) | (ycoor_n << yshift);       //write y
                    data = (data & predwmask) | (pred_n << predshift);  //write pred
                    data = (data & layerwmask) | layer_n;               //write layer
                    wf.push(data);
                    grid[xcoor][ycoor+1][layer] = grid[xcoor][ycoor+1][layer] | (pred_n << 12);   //store pred in grid
                    if(xcoor_n == xt && ycoor_n == yt && layer_n == lt) target = true;   //check if target reached
                }
            }
        }
        if(layer == 0){
            if(!(grid[xcoor][ycoor][layer+1] >> 15)){  //if not reached in above layer
                layer_n = 1;    //go to layer 2
                grid[xcoor][ycoor][layer_n] = grid[xcoor][ycoor][layer_n] | (1 << 15);    //mark cell as reached
                pc_n = (grid[xcoor][ycoor][layer_n] & 4095);   //cell cost
                if(pc_n != 0){     //proceed only if the cell is not an obstacle
                    cout << "Above Cell\n";
                    pred_n = 6;     //reached from Below
                    pc_n = pc + pc_n + via;  //old pc + cell cost + via cost
                    xcoor_n = xcoor;
                    ycoor_n = ycoor;
                    //write into WF
                    data = 0;
                    data = (data & pcwmask) | (pc_n << pcshift);        //write pathcost pc
                    data = (data & xwmask) | (xcoor_n << xshift);       //write x
                    data = (data & ywmask) | (ycoor_n << yshift);       //write y
                    data = (data & predwmask) | (pred_n << predshift);  //write pred
                    data = (data & layerwmask) | layer_n;               //write layer
                    wf.push(data);
                    grid[xcoor][ycoor][layer_n] = grid[xcoor][ycoor][layer_n] | (pred_n << 12);   //store pred in grid
                    if(xcoor_n == xt && ycoor_n == yt) target = true;   //check if target reached
                }
            }
        }
        if(layer == 1){
            if(!(grid[xcoor][ycoor][layer-1] >> 15)){  //if not reached in above layer
                layer_n = 0;    //go to layer 1
                grid[xcoor][ycoor][layer_n] = grid[xcoor][ycoor][layer_n] | (1 << 15);    //mark cell as reached
                pc_n = (grid[xcoor][ycoor][layer_n] & 4095);   //cell cost
                if(pc_n != 0){     //proceed only if the cell is not an obstacle
                    cout << "Below Cell\n";
                    pred_n = 5;     //reached from Above
                    pc_n = pc + pc_n + via;  //old pc + cell cost + via cost
                    xcoor_n = xcoor;
                    ycoor_n = ycoor;
                    //write into WF
                    data = 0;
                    data = (data & pcwmask) | (pc_n << pcshift);        //write pathcost pc
                    data = (data & xwmask) | (xcoor_n << xshift);       //write x
                    data = (data & ywmask) | (ycoor_n << yshift);       //write y
                    data = (data & predwmask) | (pred_n << predshift);  //write pred
                    data = (data & layerwmask) | layer_n;               //write layer
                    wf.push(data);
                    grid[xcoor][ycoor][layer_n] = grid[xcoor][ycoor][layer_n] | (pred_n << 12);   //store pred in grid
                    if(xcoor_n == xt && ycoor_n == yt) target = true;   //check if target reached
                }
            }
        }
    }
    counter = 0;
    if(target){
        cout << "Target Reached\n";
        xcoor = xt;
        ycoor = yt;
        layer = lt;
        while(!source){     //backtrace
            pred = (grid[xcoor][ycoor][layer] & 28672) >> 12;    //read pred
            grid[xcoor][ycoor][layer] = grid[xcoor][ycoor][layer] & 61440;    //set as obstacle
            place[counter][0] = layer;  //store coordinates of path in array
            place[counter][1] = xcoor;
            place[counter][2] = ycoor;
            counter++;
            //cout << "layer: " << layer << " x: " << xcoor << " y: " << ycoor << endl;
            if(pred == 1) ycoor += 1;        //north
            else if(pred == 2) xcoor += 1;   //east
            else if(pred == 3) ycoor -= 1;   //south
            else if(pred == 4) xcoor -= 1;   //west
            else if(pred == 5) layer = 1;    //Layer2
            else if(pred == 6) layer = 0;    //Layer1
            if(xcoor == xs && ycoor == ys) source = true;
        }
        for(int i=0; i<X; i++){             //cleanup
            for(int j=0; j<Y; j++){
                grid[i][j][0] = grid[i][j][0] & 4095;
                grid[i][j][1] = grid[i][j][1] & 4095;
            }
        }
    }
    else cout << "Could not reach Target\n";
    while( !wf.empty() ) wf.pop(); // reset the heap
    return(counter);
}

int main(){
    cout << "Begin\n";
    int data;
    unsigned int counter, prev_l;
    unsigned int net_tot, net_num, ls, xs, ys, lt, xt, yt;
    cout << "Open Grid File\n";
    ifstream gridfile;
    gridfile.open("ProgrammingAssignment4Files/bench4.grid");
    //read x, y, bend cost, via cost
    gridfile >> X;
    gridfile >> Y;
    gridfile >> bend;
    gridfile >> via;
    //cout << X << " " << Y << " " << bend << " " << via << "\n";
    xubound = X;
    yubound = Y;
    //create grid array
    grid = new short int**[X];
    for (int i = 0; i < X; i++)
	{
		grid[i] = new short int*[Y];

		for (int j = 0; j < Y; j++)
			grid[i][j] = new short int[2];
	}

	//store data in grid array
	cout << "Read Grid File\n";
	for(int i=0; i<2; i++){
        for(int j=0; j<Y; j++){
            for(int k=0; k<X; k++){
                gridfile >> data;
                if(data > 0) grid[k][j][i] = data;
                else grid[k][j][i] = 0;
            }
        }
	}
	gridfile.close();
	//Data format for grid array (16 bits):
	//bits 0-11: cell cost (0-obstacle)
	//bits 12-14: predecessor info
	//bit 15: reached (0-not reached, 1-reached)

    //place array for storing coordinates of final route
	place = new unsigned int*[X*Y];
    for (int i = 0; i < X*Y; i++)
	{
		place[i] = new unsigned int[3];
	}

    for(int i=0; i<X*Y; i++){
        place[i][0] = 0;
        place[i][1] = 0;
        place[i][2] = 0;
    }

    cout << "Open Netlist File\n";
    ifstream nlfile;
    nlfile.open("ProgrammingAssignment4Files/bench4.nl");
    nlfile >> net_tot;  //read total number of netlists
    ofstream outfile;
    outfile.open("bench4", ios::out);
    outfile << net_tot << endl;

    cout << "Read Netlist File\n";
    for(int i=0; i<net_tot; i++){
        //read source: layer, x, y and target: layer, x, y
        nlfile >> net_num >> ls >> xs >> ys >> lt >> xt >> yt;
        counter = route(ls-1, xs, ys, lt-1, xt, yt);    //route and obtain number of cells in route
        outfile << net_num << endl;;   //net number
        if(counter > 0){
            //write into file
            outfile << ls << " " << xs << " " << ys << endl;
            prev_l = ls-1;
            for(int i=0; i<counter; i++){
                if(place[counter-1-i][0] != prev_l){    //if via
                    outfile << 3 << " " << place[counter-1-i][1] << " " << place[counter-1-i][2] << endl;
                }
                outfile << place[counter-1-i][0]+1 << " " << place[counter-1-i][1] << " " << place[counter-1-i][2] << endl;
                prev_l = place[counter-1-i][0];
                place[counter-1-i][0] = 0;
                place[counter-1-i][1] = 0;
                place[counter-1-i][2] = 0;
            }
        }
        outfile << "0\n";
    }
    nlfile.close();
    outfile.close();
    return 0;
}
