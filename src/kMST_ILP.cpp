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
		IloNumArray edgesSelected(env, edges.getSize()), flowRes(env, edges.getSize());;
		cplex.getValues(edgesSelected, edges);

		if (model_type == "scf") {
			cplex.getValues(flowRes, flow_scf);
		}
		cout << "Edges:\n";

		for (unsigned int i=0; i<edges.getSize(); i++) {

			// skip unused ones
			if (((int)edgesSelected[i])  == 0 ) {
				continue;
			}

			if (i == instance.n_edges) {
				cout << endl;
			}
			bool direction = ( i >= instance.n_edges);
			cout << "  " << setw(4) <<  i << ": " << ((int)edgesSelected[i]) << " ";

			// flow
			if (model_type == "scf") {
				cout << "f: " << setw(2) << (flowRes[i]);
			}

			cout << " " << Tools::edgeToString(instance.edges[i % instance.n_edges], direction) ;

			cout << endl;
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
void kMST_ILP::getOutgoingEdgeIds(vector<u_int> & outgoingEdgeIds, u_int vertex)
{
	getVertexEdgeIds(outgoingEdgeIds, vertex, /*outgoing=*/true);
}

void kMST_ILP::getIncomingEdgeIds(vector<u_int> & incomingEdgeIds, u_int vertex)
{
	getVertexEdgeIds(incomingEdgeIds, vertex, /*outgoing=*/false);
}

void kMST_ILP::getVertexEdgeIds(vector<u_int> & edgeIds, u_int vertex, bool outgoing)
{
	const list<u_int> incidences = instance.incidentEdges[vertex];

	edgeIds.resize(incidences.size());

	int i = -1;

	for (list<u_int>::const_iterator iter = incidences.begin();
			 iter != incidences.end(); ++iter) {
		i++;

		unsigned int edgeId = *iter;
		const Instance::Edge & edge = instance.edges[edgeId];

		if ( (outgoing && edge.v1 == vertex ) || (!outgoing && edge.v2 == vertex) ) {
			edgeIds[i] = edgeId;
		} else {
			edgeIds[i] = edgeId + instance.n_edges;
		}
	}
}


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

	// exactly k edges (plus one from dummy node 0)
	model.add(IloSum(edges) == k+1);

	// no 2 incoming edges per vertex
	for (unsigned int i=0; i < instance.n_nodes; i++ ){

		vector<u_int> incomingEdgeIds;
		getIncomingEdgeIds(incomingEdgeIds, i);

		IloExpr incomingSum(env);

		for (unsigned int i=0; i < incomingEdgeIds.size(); i++ ){
			incomingSum += edges[incomingEdgeIds[i]];
		}

		model.add(incomingSum <= 1);
		incomingSum.end();
	}

	// only 1 outgoing node from 0
	{
		IloExpr outgoingSum(env);

		bool hasOutgoing = false;
		{
			vector<u_int> outgoingEdges;
			getOutgoingEdgeIds(outgoingEdges, 0);

			for (unsigned int i=0; i<outgoingEdges.size(); i++) {
				hasOutgoing = true;
				outgoingSum += edges[outgoingEdges[i]];
			}
		}

		if (hasOutgoing) {
			model.add(outgoingSum == 1);
		}
		outgoingSum.end();
	}


	// no incoming to 0
	{
		vector<u_int> incomingEdges;
		getIncomingEdgeIds(incomingEdges, 0);

		for (unsigned int i=0; i<incomingEdges.size(); i++) {
			model.add( edges[incomingEdges[i]] == 0) ;
		}

	}
}

// ----- models -----------------------------------------------

void kMST_ILP::modelSCF()
{
	// flow for each edge
	flow_scf = IloNumVarArray(env, edges.getSize());

	for (unsigned int i=0; i<flow_scf.getSize(); i++) {
		flow_scf[i] = IloNumVar(env, Tools::indicesToString("flow", i).c_str());

		// not non-zero
		model.add(0 <= flow_scf[i]);
		// at most k+1, also ensures that edge is taken if flow is non-zero
		model.add(flow_scf[i] <= (k+1)*edges[i]);
	}

	// 0 emits k+1 tokens
	{
		vector<u_int> outgoingEdgeIds;
		getOutgoingEdgeIds(outgoingEdgeIds, 0);

		IloExpr outgoingFlowSum(env);
		for (unsigned int i=0; i<outgoingEdgeIds.size(); i++) {
			outgoingFlowSum += flow_scf[ outgoingEdgeIds[i] ];
		}

		model.add(outgoingFlowSum == k+1);

		outgoingFlowSum.end();
	}

	// flow conservation, each node, which is taken, eats one (except first)
	{
		vector<u_int> outgoingEdgeIds;
		getOutgoingEdgeIds(outgoingEdgeIds, 0);


		for (unsigned int vertex=1; vertex<instance.n_nodes; vertex++) {
			vector<u_int> outgoingEdgeIds, incomingEdgeIds;

			getIncomingEdgeIds(incomingEdgeIds, vertex);

			IloExpr incomingFlowSum(env);
			IloExpr incomingEdgesSum(env);
			for (unsigned int i=0; i<incomingEdgeIds.size(); i++) {
				incomingFlowSum += flow_scf[ incomingEdgeIds[i] ];
				incomingEdgesSum += edges[ incomingEdgeIds[i] ];
			}

			getOutgoingEdgeIds(outgoingEdgeIds, vertex);
			IloExpr outgoingFlowSum(env);
			for (unsigned int i=0; i<outgoingEdgeIds.size(); i++) {
				outgoingFlowSum += flow_scf[ outgoingEdgeIds[i] ];
			}

			// vertex is part solution if one incoming vertex is used

			model.add(incomingFlowSum - outgoingFlowSum == incomingEdgesSum);

			incomingFlowSum.end();
			outgoingFlowSum.end();
		}
	}
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
