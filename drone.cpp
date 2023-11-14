// Project Identifier: 1761414855B69983BD8035097EFBD312EB0527F0
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdio.h>
#include <getopt.h>
#include "drone.h"
#include <vector>
#include <string>
#include <queue>
#include <utility>


using namespace std;

double TSP(vector<unsigned int>& tsp, vector<Locations>& vert);
long double PrimMST(vector<pair<unsigned int, unsigned int>>& mst, vector<PrimData>& primVec);
std::string getArg(const char* opt);


int main(int argc, char* argv[]) {
	std::ios_base::sync_with_stdio(false);
	cout << std::setprecision(2); //Always show 2 decimal places
	cout << std::fixed; //Disable scientific notation for large numbers


	static option long_options[] = {
	{"help", no_argument, nullptr, 'h'},
	{"mode", required_argument, nullptr, 'm'},
	{nullptr, 0, nullptr, '\0'},
	};

	int gotopt = 0;
	std::string mode = "";
	while ((gotopt = getopt_long(argc, argv, "m:h", long_options, nullptr)) != -1) {
		switch (gotopt) { //switch 
		case 'm':
			mode = getArg(optarg);
			if ((mode != "MST" && mode != "FASTTSP" && mode != "OPTTSP")) {
				cerr << "That is not a valid mode. Options are MST, FASTTSP, & OPTTSP\n"; //omit before grading
			}
			break;
		case 'h':
			cout << "This program finds the most efficient path. Use the --mode flag & a valid mode\n";
			exit(0);
		default:
			cerr << "You need to provide the -m flag & a valid mode (MST, FASTTSP, OPTTSP)\n"; //omit b4 grading
			exit(1);
			break;
		} //switch 
	} //while 

	double totalWeight = 0;
	int coordx = 0;
	int coordy = 0;
	vector<Locations> vert; //vertex number = vector index. 
	vector<pair<unsigned int, unsigned int>> mst;  //MST container for output
	vector<unsigned int> tsp; // TSP container for output
	vector<PrimData> primsVec; // Vector holding metadata for Prims alg. 

	cin >> coordx; // read number of vertices to be read 
	vert.reserve(coordx); // reserve space in vector<Locations> 
	while (cin >> coordx) { // for all input regardless of mode
		cin >> coordy;
		Locations v(coordx, coordy); 
		vert.push_back(v); 
	}

	if (mode == "MST") {
		primsVec.reserve(vert.size()); // reserve space in vector of PrimsData
		mst.reserve(vert.size()); // reserve sufficient space for our MST to be generated
		for (unsigned int i = 0; i < vert.size(); i++) {
			PrimData p(vert[i].x, vert[i].y); // custom constrcutor sets x,y, and visited to false; 
			primsVec.push_back(p); // push into vector<PrimData> 
		}
		bool med = false;
		bool bord = false; 
		for (unsigned int i = 0; i < vert.size(); i++) {
			if (med && bord) {
				break;
			}
			else if (vert[i].place == Campus::Medical) {
				med = true; 
			}
			else if (vert[i].place == Campus::Border) {
				bord = true; 
			}
		}
		if (!(med && !bord)) { // there are both med and border locations, or there are neither 
			totalWeight = (double)PrimMST(mst, primsVec); //make MST
			cout << totalWeight << '\n';
			for (unsigned int i = 0; i < mst.size(); i++) {
				if (mst[i].first > mst[i].second) {
					cout << mst[i].second << ' ' << mst[i].first << '\n';
				}
				else {
					cout << mst[i].first << ' ' << mst[i].second << '\n';
				}
			}
		}
		else {
			cerr << "Cannot construct mst'\n";
			exit(1);
		}
	}// if mode == MST
	else if (mode == "FASTTSP") {
		tsp.reserve(vert.size() + 1); 
		//  initializing tour with 0-1-2-0 
		tsp.emplace_back(0);
		tsp.emplace_back(1);
		tsp.emplace_back(2);
		tsp.emplace_back(0); 
		totalWeight = TSP(tsp, vert); 
		cout << totalWeight << '\n'; 
		for (unsigned int i = 0; i < tsp.size() - 1; i++) {
			cout << tsp[i] << ' ';
		}
	}

	return 0;
} //main 

double TSP(vector<unsigned int>& tsp, vector<Locations>& vert) {
	unsigned int arb = 3; 
	double min_delta = numeric_limits<double>::infinity(), delta = 0;
	Locations k; 
	vector<unsigned int>::iterator addHere = tsp.begin(); 
	while ((tsp.size() != vert.size() + 1) && arb < vert.size()) { // STOP when all indices have been added [V + 1] b/c start/end node

		k = vert[arb]; // Our current 'arbitrary vertex' 
		min_delta = numeric_limits<double>::infinity(); // reset for comparisons 
		for (vector<unsigned int>::iterator it = tsp.begin(); it != (tsp.end() - 1); it++) { // finding best improvement to make - don't go off edge
			delta = k.distance(vert[*it].x, vert[*it].y) + k.distance(vert[*(it + 1)].x, vert[*(it + 1)].y)  // C[ik - Ckj - Cij]
					- vert[*it].distance(vert[*(it + 1)].x, vert[*(it + 1)].y); 
			if (delta < min_delta) {
				min_delta = delta;
				addHere = (it + 1);
			}
		}
		tsp.insert(addHere, arb); // inserting the index from smallest delta combo - ks index is arb 
		arb++; 
	}
	delta = 0; // for summing below 
	for (vector<unsigned int>::iterator it = tsp.begin(); it != (tsp.end() - 1); it++){
		delta += vert[*it].distance(vert[*(it + 1)].x, vert[*(it + 1)].y);; // calculating total weight; 
	}
	return delta; 
}

long double PrimMST(vector<pair<unsigned int, unsigned int>>& mst, vector<PrimData>& primVec) { //repeat until all visited == true

	primVec[0].weight = 0;
	primVec[0].parent = -1; 
	int count = (int)primVec.size();
	int currNode = 0;
	double dist = 0, minDist = numeric_limits<double>::infinity(); 
	double totalW = 0; 
	unsigned int index = 0;
	vector<unsigned int> avail;
	for (unsigned int i = 0; i < primVec.size(); i++) {
		avail.push_back(i);
	}
	while (count != 0) {
		minDist = numeric_limits<double>::infinity(); 
		for (unsigned int i = 0; i < avail.size(); i++) {
			if (primVec[avail[i]].weight < minDist) { // finding smallest Dv value 
				minDist = primVec[avail[i]].weight;
				currNode = avail[i]; // found our new  target, now let's update distances relative to here 
				index = i; 
			}
		}
		primVec[currNode].visited = true; 
		std::swap(avail[index], avail.back()); 
		avail.pop_back(); 
		if (count != (int)primVec.size()) {
			mst.push_back(make_pair((unsigned int)currNode, (unsigned int)primVec[currNode].parent));
		}
		count--; 
		totalW += sqrt(minDist); 
		for (unsigned int i = 0; i < avail.size(); i++) { // updating distances 
			dist = primVec[currNode].Euclid(primVec[avail[i]].x, primVec[avail[i]].y);
			if (primVec[avail[i]].weight > dist) { // checks if distance is smaller between recently...
				primVec[avail[i]].weight = dist;	// ...added node and this point, than any other point in our mst 
				primVec[avail[i]].parent = currNode;
			}
		}
	}
	return (long double)totalW; 
}

std::string getArg(const char* opt) {
	std::string str(opt);
	return str;
}