// Project Identifier: 1761414855B69983BD8035097EFBD312EB0527F0
#include<vector> 
#include <math.h>
using namespace std; 

enum class Campus : char {Medical, Border, Normal, Other};

struct Locations {
	int x = 0; 
	int y = 0;
	Campus place = Campus::Other;
	Locations() = default; 
	Locations(int a, int b) {
		x = a;
		y = b;
		if (x < 0 && y < 0) {
			place = Campus::Medical;
		}
		else if ((x <= 0 && y == 0) || (x == 0 && y < 0)) {
			place = Campus::Border;
		}
		else {
			place = Campus::Normal;
		}
	}
	double distance(int a, int b) {
		double deltax = (double)abs(x - a);
		double deltay = (double)abs(y - b); 
		return sqrt((deltax * deltax) + (deltay * deltay));
	}

	Locations& operator=(Locations& rhs) {
		x = rhs.x;
		y = rhs.y;
		return *this; 
	}

};

struct PrimData {
	double weight = std::numeric_limits<double>::infinity();
	int x = 0, y = 0, parent = 0;
	bool visited = false; 
	PrimData(int a, int b) { // custom constructor 
		x = a;
		y = b;
	}
	PrimData() = default; 
	double Euclid(int a, int b) {
		Locations lhs(x, y); 
		Locations rhs(a, b); 
		if (lhs.place == Campus::Medical && rhs.place == Campus::Normal) { // check valid move if not medical-medical or medical-border
				return std::numeric_limits<double>::infinity(); // can't be done, return infinity()
		}
		else if (rhs.place == Campus::Medical && lhs.place == Campus::Normal) { // check valid move
				return std::numeric_limits<double>::infinity(); 
		}
		else { // calc and return distance 
			return (double)(((double)(abs(x - a))*(double)(abs(x - a))) + ((double)(abs(y - b)) * (double)(abs(y - b)))); // return sum of their (positive differences)^2, magnitude will be same. 
		}
		return (double)(((double)(abs(x - a)) * (double)(abs(x - a))) + ((double)(abs(y - b)) * (double)(abs(y - b)))); // return sum of their (positive differences)^2, magnitude will be same. 
	}

};









