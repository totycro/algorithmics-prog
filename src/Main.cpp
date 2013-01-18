#ifndef __MAIN__CPP__
#define __MAIN__CPP__

#include <iostream>
#include <fstream>
#include "Tools.h"
#include "Instance.h"
#include "kMST_ILP.h"

using namespace std;

void usage()
{
	cout << "USAGE:\t<program> -f filename -m model [-k <nodes to connect> -l <logfile>]\n";
	cout << "EXAMPLE:\t" << "./kmst -f data/g01.dat -m scf -k 5 -l log.txt\n\n";
	exit( 1 );
} // usage

int main( int argc, char *argv[] )
{
	// read parameters
	int opt;
	// default values
	string file( "data/g01.dat" );
	string model_type( "flow" );
	int k = 5;
	bool doLogging = false;
	string logFilename("");
	int rounds = 1;
	while( (opt = getopt( argc, argv, "f:m:k:l:r:" )) != EOF) {
		switch( opt ) {
			case 'f': // instance file
				file = optarg;
				break;
			case 'm': // algorithm to use
				model_type = optarg;
				break;
			case 'k': // nodes to connect
				k = atoi( optarg );
				break;
			case 'l': // logfile
				logFilename = optarg;
				doLogging = true;
				break;	
			case 'r': // rounds of executions
				rounds = atoi( optarg );
				break;	
			default:
				usage();
				break;
		}
	}
	// read instance
	Instance instance( file );
	// solve instance
	double objectiveValue = 0;
	int nodes = 0;
	
	cerr << "Executing " << rounds << " rounds of " << file << " with " << model_type << " k=" << k << "\r\n";

	for (int round=0; round<rounds; round++) {
		kMST_ILP ilp( instance, model_type, k );
		ilp.solve();
		objectiveValue = ilp.getObjectiveValue();
		nodes = ilp.getNodes();
	}

	// log results
	if (doLogging) {
		bool exists = false;
		ifstream logIn(logFilename.c_str());
		if (logIn) {
			exists = true;
		}
		logIn.close();

		ofstream log;
		log.open(logFilename.c_str(),ios::ate | ios::app);
 		if (!exists) {
			log <<"Filename\tModel\tNodes\tCost\tB&B N\tCPUTime\r\n";
		}
		log <<file <<"\t"<< model_type <<"\t"<< k <<"\t"<< 
			objectiveValue <<"\t"<< nodes
			<<"\t"<< Tools::CPUtime()/ rounds<<"\r\n";
 		log.close();
	}

	return 0;
} // main

#endif // __MAIN__CPP__
