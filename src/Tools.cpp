#ifndef __TOOLS__CPP__
#define __TOOLS__CPP__

#include "Tools.h"

#include <deque>

string Tools::indicesToString( string prefix, int i, int j, int v )
{
	stringstream ss;
	ss << prefix << "(" << i;
	if( j >= 0 ) ss << ',' << j;
	if( v >= 0 ) ss << ',' << v;
	ss << ')';
	return ss.str();
}

string Tools::edgeToString(const Instance::Edge& edge, bool direction)
{
	stringstream ss;
	ss << "edge(" ;
	if (!direction) {
		ss << setw(3) << edge.v1 << " to " << setw(3) << edge.v2 ;
	} else {
		ss << setw(3) << edge.v2 << " to " << setw(3) << edge.v1 ;
	}
	ss << ", w:" << setw(2) << edge.weight << ")";
	return ss.str();
}


double Tools::CPUtime()
{
	tms t;
	times( &t );
	double ct = sysconf( _SC_CLK_TCK );
	return t.tms_utime / ct;
}

Tools::Tree::Tree(int sz) : tree(sz)
{
	//cerr << "\nsz: "<<sz <<"\n" << endl;
	for (int i=0; i<sz; i++) {
		tree[i] = list<pair<int, float> >();
	}
}

void Tools::Tree::addEdge(int i, int j, float flow)
{
	//cerr << "\ni: "<<i <<"\n" << endl;
	tree[i].push_back( pair<int, float>(j, flow) );
}

void Tools::Tree::print(ostream& s)
{
	s << "digraph G {\n";
	for (unsigned int i=0; i<tree.size(); i++) {
		if (!tree[i].empty()) {
			s << "\t" << i << ";\n";
		}
	}

	deque<int> queue;
	queue.push_back(0);
	while (!queue.empty()) {
		int node = queue.front();
		queue.pop_front();

		const list<pair<int, float> > & neightbors = tree[node];

		//s << "Node " << node << "\n";

		for (list<pair<int, float> >::const_iterator iter = neightbors.begin();
				 iter != neightbors.end(); iter++) {
			const pair<int, float> & neighbor = *iter;
			queue.push_back(neighbor.first);
			//s << " to " << neighbor.first << ", flow: " << neighbor.second << "\n";

			s << "\t" << node << " -> " <<  neighbor.first << " [ label = \"" << neighbor.second << "\" ] ;\n";

		}
	}

	s << "}\n";

}


#endif // __TOOLS__CPP__
