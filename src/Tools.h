#ifndef __TOOLS__H__
#define __TOOLS__H__

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <ctime>
#include <stdexcept>
#include <vector>
#include <string>
#include <sstream>
#include <algorithm>
#include <iomanip>
#include <sys/times.h>
#include "Instance.h"

using namespace std;

namespace Tools
{
	// generate string from edge indices
	string indicesToString( string prefix, int i, int j = -1, int v = -1 );

	string edgeToString(const Instance::Edge & edge, bool direction);

	// measure running time
	double CPUtime();

	struct Tree {
		vector<list<pair<int, float> > > tree;
		Tree(int sz);
		void addEdge(int i, int j, float flow);
		void print(ostream&);
	};
}
;
// Tools

#endif // __TOOLS__H__
