#include "Instance.h"

Instance::Instance( string file )
{
	ifstream ifs( file.c_str() );
	if( ifs.fail() ) {
		cerr << "could not open input file " << file << "\n";
		exit( -1 );
	}

	cout << "Reading instance from file " << file << "\n";

	ifs >> n_nodes >> n_edges;
	cout << "Number of nodes: " << n_nodes << "\n";
	cout << "Number of edges: " << n_edges << "\n";

	edges.resize( n_edges );
	incidentEdges.resize( n_nodes );

	u_int id;
	while( ifs >> id ) {
		ifs >> edges[id].v1 >> edges[id].v2 >> edges[id].weight;
		incidentEdges[edges[id].v1].push_back( id );
		incidentEdges[edges[id].v2].push_back( id );
	}
	ifs.close();

//	cout << "Incidency list:" << "\n";
//	for( u_int v = 0; v < n_nodes; v++ ) {
//		cout << v << ": ";
//		for( list<u_int>::iterator li = incidentEdges[v].begin();
//				li != incidentEdges[v].end(); li++ ) {
//			cout << "(" << edges[*li].v1 << "," << edges[*li].v2 << "), ";
//		}
//		cout << "\n";
//	}
}
