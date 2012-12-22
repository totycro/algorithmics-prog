#include "kMST_ILP.h"

kMST_ILP::kMST_ILP( Instance& _instance, string _model_type, int _k ) :
		instance( _instance ), model_type( _model_type ), k( _k )
{
	n = instance.n_nodes;
	m = instance.n_edges;
	if( k == 0 ) k = n;
}

void kMST_ILP::solve()
{
	try {
		// initialize CPLEX
		env = IloEnv();
		model = IloModel( env );

		addTreeConstraints(); // call first, initialises edges

		// add model-specific constraints
		if( model_type == "scf" ) modelSCF();
		else if( model_type == "mcf" ) modelMCF();
		else if( model_type == "mtz" ) modelMTZ();
		else {
			cerr << "No existing model chosen\n";
			exit( -1 );
		}

		addObjectiveFunction();

		// build model
		cplex = IloCplex( model );
		// export model to a text file
		//cplex.exportModel( "model.lp" );
		// set parameters
		setCPLEXParameters();

		// solve model
		cout << "Calling CPLEX solve ...\n";
		cplex.solve();
		cout << "CPLEX finished.\n\n";
		cout << "CPLEX status: " << cplex.getStatus() << "\n";
		cout << "Branch-and-Bound nodes: " << cplex.getNnodes() << "\n";
		cout << "Objective value: " << cplex.getObjValue() << "\n";
		cout << "CPU time: " << Tools::CPUtime() << "\n\n";

		// show result
		IloNumArray edgesSelected(env, edges.getSize());
		cplex.getValues(edgesSelected, edges);
		cout << "Edges:\n";

		for (unsigned int i=0; i<edges.getSize(); i++) {
			if (i == instance.n_edges) {
				cout << endl;
			}
			bool direction = ( i >= instance.n_edges);
			cout << "  " << setw(2) <<  i << setw(0) << ": " << edgesSelected[i] << " " <<
				Tools::edgeToString(instance.edges[i % instance.n_edges], direction) << "\n";
		}
	}
	catch( IloException& e ) {
		cerr << "kMST_ILP: exception " << e << "\n";
		exit( -1 );
	}
	catch( ... ) {
		cerr << "kMST_ILP: unknown exception.\n";
		exit( -1 );
	}
}

// ----- private methods -----------------------------------------------

void kMST_ILP::setCPLEXParameters()
{
	// print every x-th line of node-log and give more details
	cplex.setParam( IloCplex::MIPInterval, 1 );
	cplex.setParam( IloCplex::MIPDisplay, 2 );
	// only use a single thread
	cplex.setParam( IloCplex::Threads, 1 );
}



// ----- private utility -----------------------------------------------

void kMST_ILP::addObjectiveFunction()
{
	// multiply variable by cost
	IloIntArray edgeCost(env, instance.n_edges * 2);

	for (unsigned int i=0; i<edges.getSize(); i++) {
		edgeCost[i] = instance.edges[i % instance.n_edges].weight;
	}

	model.add(IloMinimize(env,  IloScalProd(edges, edgeCost) ));
}

void kMST_ILP::addTreeConstraints()
{
	edges = IloBoolVarArray(env, instance.n_edges * 2); // edges in one direction and in other

	// "to"-edges on lower indices
	for (unsigned int i=0; i<instance.n_edges; i++) {
		edges[i] = IloBoolVar(env, Tools::indicesToString("edge " , instance.edges[i].v1, instance.edges[i].v2, instance.edges[i].weight).c_str() );
	}

	// edges in other direction
	for (unsigned int i=instance.n_edges; i<instance.n_edges*2; i++) {
		edges[i] = IloBoolVar(env, Tools::indicesToString("edge " , instance.edges[i].v2, instance.edges[i].v1, instance.edges[i].weight).c_str() );
	}

	// edges in one direction forbid edges in other direction
	for (unsigned int i=0; i<instance.n_edges; i++) {
		model.add( edges[i] + edges[i + instance.n_edges] <= 1 );
	}

	// exactly k edges
	model.add(IloSum(edges) == k);
};

// ----- models -----------------------------------------------


void kMST_ILP::modelSCF()
{


}

void kMST_ILP::modelMCF()
{
	// ++++++++++++++++++++++++++++++++++++++++++
	// TODO build multi commodity flow model
	// ++++++++++++++++++++++++++++++++++++++++++
}

void kMST_ILP::modelMTZ()
{
	// ++++++++++++++++++++++++++++++++++++++++++
	// TODO build Miller-Tucker-Zemlin model
	// ++++++++++++++++++++++++++++++++++++++++++
}

kMST_ILP::~kMST_ILP()
{
	// free global CPLEX resources
	cplex.end();
	model.end();
	env.end();
}
