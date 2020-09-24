# Simple-Maze-Router

Simple Maze Router for routing on ASICs. 
The simple maze router uses breadth-first-search to find the minimum cost path from a source to its target. It is implemented for 2-point nets and upto 2 routing layers. 
The routing surface is assumed to be an NxM grid with cells of uniform size. Each cell has a cost associated with it. Routing can be avoided in a certain area of the grid by associating higher cell costs with the cells. Routing can be prevented in a certain area of the grid by setting the cells as obstacles.

It requires two input files: the grid file and the netlist file.
The grid file is in the following format:
First line: <number of coordinates in the x direction (X), number of coordinates in the y direction (Y), bend penalty, via penalty>
Next Y lines give the cell cost of each cell in layer 1 (cell cost is above 0; if its an obstacle, cell cost = -1)
Last Y lines give the same information for layer 2.
The netlist file is in the following format:
First line: number of nets to route (N)
Next N lines give the information for the nets to be routed –  <layer of source, x coordinate of source, y coordinate of source, layer of target, x coordinate of target, y coordinate of target> 
Output file: First line gives the total number of nets. 
For each net, the code writes the net number, <layer, x coordinate, y coordinate> of each cell in the route from source to target. ‘0’ marks the end of the route.
An example of the grid file, netlist file and output file is given. 

Features:
It implements both uniform cost and non-uniform cost cells
It takes into account bend penalty. 
It takes into account via penalty. 
It is implemented for only 2 layers. However more layers can be added by increasing the dimensions of the Grid matrix in the code. 
