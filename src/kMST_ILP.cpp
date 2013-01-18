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
		bool DO_LOGGING = false;
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

		// turn off logging
		if (!DO_LOGGING) {
			cplex.setOut(env.getNullStream());
		}

		// solve model
		cout << "Calling CPLEX solve ...\n";
		cplex.solve();
		cout << "CPLEX finished.\n\n";

		nodes = cplex.getNnodes();
		objectiveValue = cplex.getObjValue();

		cout << "CPLEX status: " << cplex.getStatus() << "\n";
		cout << "Branch-and-Bound nodes: " << nodes << "\n";
		cout << "Objective value: " << objectiveValue  << "\n";
		cout << "CPU time: " << Tools::CPUtime() << "\n\n";

		// show result
		IloNumArray edgesSelected(env, edges.getSize()), flowRes(env, edges.getSize()), uRes(env, instance.n_nodes);
		cplex.getValues(edgesSelected, edges);

		if (model_type == "scf") {
			cplex.getValues(flowRes, flow_scf);
		} else if (model_type == "mtz") {
			try {
				cplex.getValues(uRes, u);
			} catch ( IloException& e ) {
				cerr << "Exception while extracting u: " << e << endl;
				uRes = IloNumArray(env, 0);
			}
		}
		if (DO_LOGGING) {
		cout << "Edges:\n";

		stringstream edgeLabels; // for debug output MCF
		edgeLabels << "EDGE";

		Tools::Tree tree(instance.n_nodes);

		for (unsigned int i=0; i<edges.getSize(); i++) {

			edgeLabels << " " << instance.edges[i % instance.n_edges].v1 << instance.edges[i % instance.n_edges].v2;

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
			} else if (model_type == "mtz") {
				if (uRes.getSize() != 0) {
					cout << "u: " ;
					if (i < instance.n_edges) {
						//cout << setw(2) << instance.edges[i % instance.n_edges].v1 << ": ";
						cout << setw(2) <<((int)uRes[ instance.edges[i % instance.n_edges].v1]) << " ";
						//cout << setw(2) << instance.edges[i % instance.n_edges].v2 << ": ";
						cout << setw(2) << ((int)uRes[ instance.edges[i % instance.n_edges].v2]) ;
					}  else {
						//cout << setw(2) << instance.edges[i % instance.n_edges].v2 << ": ";
						cout << setw(2) <<((int)uRes[ instance.edges[i % instance.n_edges].v2]) << " ";
						//cout << setw(2) << instance.edges[i % instance.n_edges].v1 << ": ";
						cout << setw(2) <<((int)uRes[ instance.edges[i % instance.n_edges].v1]) ;
					}
				}
			} else if (model_type == "mcf") {

			}

			cout << " " << Tools::edgeToString(instance.edges[i % instance.n_edges], direction) ;


			cout << endl;


			if (model_type == "scf") { // build tree
				if (!direction) {
					tree.addEdge( instance.edges[i % instance.n_edges].v1,
												instance.edges[i % instance.n_edges].v2,
												flowRes[i] );
				} else {
					tree.addEdge( instance.edges[i % instance.n_edges].v2,
												instance.edges[i % instance.n_edges].v1,
												flowRes[i] );
				}
			}
		}
		if (false) {
			cout << "\nscf-tree: \n";
			tree.print(cout);
		}

		}// do logging





	} catch (IloAlgorithm::CannotExtractException& e) {
		cerr << "CannotExtractException: " << e << endl ;
		IloExtractableArray failed = e.getExtractables();
		for (IloInt i = 0; i < failed.getSize(); ++i) {
			cerr << "\t" << failed[i] << std::endl;
		}
	} catch( IloException& e ) {
		cerr << "kMST_ILP: exception " << e << "\n";
		exit( -1 );
	}
	catch( ... ) {
		cerr << "kMST_ILP: unknown exception.\n";
		exit( -1 );
	}
}

// getter Methodss
int kMST_ILP::getNodes() {
	return nodes;
}

double kMST_ILP::getObjectiveValue() {
	return objectiveValue;
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
		//cerr << "init: edge " << i << " from " << instance.edges[i].v1 << " to " << instance.edges[i].v2 << endl;
	}

	// edges in other direction
	for (unsigned int i=instance.n_edges; i<instance.n_edges*2; i++) {
		Instance::Edge edgeInst = instance.edges[ i % instance.n_edges ];
		edges[i] = IloBoolVar(env, Tools::indicesToString("edge " , edgeInst.v2, edgeInst.v1, edgeInst.weight).c_str() );
		//cerr << "init: edge " << i << " from " << instance.edges[i%instance.n_edges].v2 << " to " << instance.edges[i%instance.n_edges].v1 << endl;
	}

	// edges in one direction forbid edges in other direction
	for (unsigned int i=0; i<instance.n_edges; i++) {
		model.add( edges[i] + edges[i + instance.n_edges] <= 1 );
	}

	// exactly k nodes, so k-1 actual edges plus one to the pseudo node 0
	model.add(IloSum(edges) == k);

	// no 2 incoming edges per vertex
	for (unsigned int i=0; i < instance.n_nodes; i++ ){

		IloExpr incomingSum(env);

		{
			vector<u_int> incomingEdgeIds;
			getIncomingEdgeIds(incomingEdgeIds, i);


			for (unsigned int i=0; i < incomingEdgeIds.size(); i++ ){
				incomingSum += edges[incomingEdgeIds[i]];
			}
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

		if (hasOutgoing) { // only true for special input files
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
	// single commodity flow model

	// flow for each edge
	flow_scf = IloNumVarArray(env, edges.getSize()); 

	for (unsigned int i=0; i<flow_scf.getSize(); i++) {
		flow_scf[i] = IloNumVar(env, Tools::indicesToString("flow", i).c_str());
		//flow_scf[i] = IloNumVar(env, 0, IloInfinity, IloNumVar::Int, Tools::indicesToString("flow", i).c_str());

		// non-zero
		model.add(0 <= flow_scf[i]);

		// max possible flow is k for the connection from the artificial root to the real node
		// the other ones then can carry a maximum of k-1, since the real root eats the first one

		#ifdef STRENGTHEN_CONSTRAINTS
		bool firstHalf = ( i < instance.n_edges);
		int startNode = firstHalf ? instance.edges[i].v1 : instance.edges[i - instance.n_edges].v2;
		int maxFlowOnEdge = (startNode == 0) ? k : k-1;
		#else
		int maxFlowOnEdge = k;
		#endif

		// at most k, also ensures that edge is taken if flow is non-zero
		model.add(flow_scf[i] <= maxFlowOnEdge*edges[i]);
	}

	// 0 emits k tokens
	{
		vector<u_int> outgoingEdgeIds;
		getOutgoingEdgeIds(outgoingEdgeIds, 0);

		IloExpr outgoingFlowSum(env);
		for (unsigned int i=0; i<outgoingEdgeIds.size(); i++) {
			outgoingFlowSum += flow_scf[ outgoingEdgeIds[i] ];
		}

		model.add(outgoingFlowSum == k);
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
			incomingEdgesSum.end();
			outgoingFlowSum.end();
		}
	}
}

void kMST_ILP::modelMCF()
{
	//  multi commodity flow model

	vector<IloBoolVarArray> flow;
	// flow for each edge and commodity
	flow.resize(instance.n_nodes);

	for (unsigned int j=0; j<flow.size(); j++) { //  commodity j
		flow[j] = IloBoolVarArray(env, edges.getSize());
		for (unsigned int i=0; i<flow[j].getSize(); i++) { // edge i

			Instance::Edge edgeInst = instance.edges[ i % instance.n_edges ];

			uint start = edgeInst.v1, end = edgeInst.v2;
             		if (i >= instance.n_edges) { // upper half
				uint tmp = start;
				start = end;
				end = tmp;
			}

			flow[j][i] = IloBoolVar(env, Tools::indicesToString("f", start, end).c_str());
			
			#ifdef STRENGTHEN_CONSTRAINTS
			// strenghten:no flow back to root
			if (end == 0) {
				flow[j][i].setUB(0);
			}
			#endif

 		}
	}

	// preperation

	IloExprArray incomingEdgesSum = IloExprArray(env, instance.n_nodes);

	for (unsigned int commodity=0; commodity<flow.size(); commodity++) { //  commodity k for vertex k
		vector<u_int> incomingEdgeIds, outgoingEdgeIds;
		getIncomingEdgeIds(incomingEdgeIds, commodity);

		incomingEdgesSum [commodity] = IloExpr(env);

		for (unsigned int i=0; i<incomingEdgeIds.size(); i++) {
			incomingEdgesSum[commodity] += edges[ incomingEdgeIds[i] ];
		}


	}


	// node 0 emits k-1 different commodities
       // tries all n-1, but only sends to nodes with incoming edge
	{
		vector<u_int> outgoingEdgeIds;
		getOutgoingEdgeIds(outgoingEdgeIds, 0);

		for (unsigned int commodity=1; commodity<flow.size(); commodity++) { //  commodity k for vertex k
			IloExpr outgoingFlowSum(env);
			for (unsigned int i=0; i<outgoingEdgeIds.size(); i++) {
				outgoingFlowSum += flow[ commodity ][ outgoingEdgeIds[i] ];
			}

			// only send out commodity for those vertices
			//that a part of the k-MST
			model.add(outgoingFlowSum == incomingEdgesSum[commodity]);

			outgoingFlowSum.end();


 		}
	}

	// each vertex recieves his commodity (if part of k-MST)
		for (unsigned int commodity=1; commodity<flow.size(); commodity++) { //  commodity k for vertex k
			vector<u_int> incomingEdgeIds;
			getIncomingEdgeIds(incomingEdgeIds, commodity);
			IloExpr incomingFlowSum(env);
			for (unsigned int i=0; i<incomingEdgeIds.size(); i++) {
				incomingFlowSum += flow[ commodity ][ incomingEdgeIds[i] ];
			}

			model.add(incomingFlowSum == incomingEdgesSum[commodity]);

			incomingFlowSum.end();
		}


	// each vertex forwards all other commodities
	for (unsigned int vertex=1; vertex<flow.size(); vertex++) {
		vector<u_int> outgoingEdgeIds, incomingEdgeIds; 
		getIncomingEdgeIds(incomingEdgeIds, vertex); // these are different from the edges above
		getOutgoingEdgeIds(outgoingEdgeIds, vertex);

		for (unsigned int commodity=1; commodity<flow.size(); commodity++) { //  commodity k for vertex k

			IloExpr incomingFlowSum(env);
			for (unsigned int i=0; i<incomingEdgeIds.size(); i++) {
				incomingFlowSum += flow[ commodity ][ incomingEdgeIds[i] ];
			}

			IloExpr outgoingFlowSum(env);
			for (unsigned int i=0; i<outgoingEdgeIds.size(); i++) {
				outgoingFlowSum += flow[ commodity ][ outgoingEdgeIds[i] ];
			}
			if (vertex != commodity) {
				model.add(incomingFlowSum - outgoingFlowSum == 0);
			}
			//incomingFlowSum.end();
			outgoingFlowSum.end();
		}

 	}




	// restrict flow to selected edges
	for (unsigned int edgeId=0; edgeId < edges.getSize(); edgeId++) {
		for (unsigned int commodity=1; commodity<flow.size(); commodity++) { //  commodity k for vertex k
			model.add ( flow[commodity][edgeId] <= edges[edgeId] );
		}
	}

	for (unsigned int commodity=1; commodity<flow.size(); commodity++) { //  commodity k for vertex k
		incomingEdgesSum[commodity].end();
	}

	// copy for former access
	flow_mcf = flow;

}


void kMST_ILP::modelMTZ()
{
	// Miller-Tucker-Zemlin model

	IloInt u_max = k;//instance.n_nodes;

	// some u_i for each vertex
	u = IloIntVarArray(env, instance.n_nodes);
	for (unsigned int i=0; i<u.getSize(); i++) {
		u[i] = IloIntVar(env, 0, u_max, Tools::indicesToString("u", i).c_str());

		#ifdef STRENGTHEN_CONSTRAINTS
		// strengthen constraints for non artificial nodes
		if (i > 0) {
			u[i].setLB(1); // TODO benchmark
		}
		#endif
	}

	// 0 vertex has fixed value
	u[0].setUB(0);

	// if there is a connection x_ij, the u_j is greater than u_i
	for (unsigned int edgeId=0; edgeId < edges.getSize(); edgeId++) {
		Instance::Edge edgeInst = instance.edges[ edgeId % instance.n_edges ];

		uint start = edgeInst.v1, end = edgeInst.v2;
             	if (edgeId >= instance.n_edges) { // upper half
			uint tmp = start;
			start = end;
			end = tmp;
		}

		//cerr << "edge " << edgeId << " from " << start << " to " << end << endl;

		// u_start + edge < u_end + (1-edge) * M
		model.add( (u[start] + edges[edgeId])  - u[end] - ( ( 1 - edges[edgeId]) * u_max )  <= 0 );

		// TODO u = 1 for first node

		if (start == 0) {
			// help u assignment: we know that if a an edge leaving 0 is chosen, the u-value has to be 1 for the node entered by the edge

			// NOTE: this should be used for big instances (e.g. g06), but leads to worse runtimes for smaller instances

			model.add( u[end] <= edges[edgeId] + ( (1 - edges[edgeId]) * u_max  ) );

		}


	}

	#ifdef STRENGTHEN_CONSTRAINTS
	IloExpr uSum(env);
	

	// if there are no incoming edges to a vertex, its u_i is maximal
	// this prevents any outgoing edges
	// the condition is not stated for node 0 to allow a start
	for (unsigned int vertex=1; vertex<instance.n_nodes; vertex++) {
		IloExpr incomingEdgesSum(env);

		{
			vector<u_int> incomingEdgeIds;
			getIncomingEdgeIds(incomingEdgeIds, vertex);

			for (unsigned int i=0; i<incomingEdgeIds.size(); i++) {
				incomingEdgesSum += edges[ incomingEdgeIds[i] ];
			}
		}
		#ifdef STRENGTHEN_CONSTRAINTS
		uSum += u[vertex];
		#endif
		// if there are no incoming edges, the subtrahend is 0, so u_vertex is forced to be maximal
		// if there are incoming edges, the lhs is smaller or equal to 0, so the condition doesn't go into effect
		model.add( (u_max - (incomingEdgesSum * u_max)) <= u[vertex]);

		// this is the same but probably more efficient, maybe test with it:
		//model.add( IloIfThen(env, incomingEdgesSum == 0, u[vertex] == u_max) );

		incomingEdgesSum.end();
	}

	#ifdef STRENGTHEN_CONSTRAINTS
	//strengthening conntraint to better describe distribution of u values
	// we wanted alldifferent(exponentially many constraints), but this has to suffice
	int sumOverU = u_max * ( instance.n_nodes -1 - k ) + // all unconnected nodes
		    k * (k+1) / 2; //small gauss	
	model.add( uSum <= sumOverU);
	#endif 
}

/* original version, results in excellent values for 07/60, but is worse for all others
void kMST_ILP::modelMTZ()
{
	// Miller-Tucker-Zemlin model

	IloInt u_max = instance.n_nodes;

	// some u_i for each vertex
	u = IloIntVarArray(env, instance.n_nodes);
	for (unsigned int i=0; i<u.getSize(); i++) {
		u[i] = IloIntVar(env, 0, u_max, Tools::indicesToString("u", i).c_str());
	}

	// 0 vertex has fixed value
	u[0].setUB(0);

	// if there is a connection x_ij, the u_j is greater than u_i
	for (unsigned int edgeId=0; edgeId < edges.getSize(); edgeId++) {
		Instance::Edge edgeInst = instance.edges[ edgeId % instance.n_edges ];
		uint start = edgeInst.v1, end = edgeInst.v2;

		if (edgeId >= instance.n_edges) { // upper half
			uint tmp = start;
			start = end;
			end = tmp;
		}

		//cerr << "edge " << edgeId << " from " << start << " to " << end << endl;

		// u_start + edge < u_end + (1-edge) * M
		model.add( (u[start] + edges[edgeId])  - u[end] - ( ( 1 - edges[edgeId]) * u_max )  <= 0 );
	}

	// if there are no incoming edges to a vertex, its u_i is maximal
	// this prevents any outgoing edges
	// the condition is not stated for node 0 to allow a start
	for (unsigned int vertex=1; vertex<instance.n_nodes; vertex++) {
		IloExpr incomingEdgesSum(env);

		{
			vector<u_int> incomingEdgeIds;
			getIncomingEdgeIds(incomingEdgeIds, vertex);

			for (unsigned int i=0; i<incomingEdgeIds.size(); i++) {
				incomingEdgesSum += edges[ incomingEdgeIds[i] ];
			}
		}

		// if there are no incoming edges, the subtrahend is 0, so u_vertex is forced to be maximal
		// if there are incoming edges, the lhs is smaller or equal to 0, so the condition doesn't go into effect
		// NOTE: this is a quite loose model
		model.add( (u_max - (incomingEdgesSum * u_max)) <= u[vertex]);

		// this is the same but probably more efficient, maybe test with it:
		//model.add( IloIfThen(env, incomingEdgesSum == 0, u[vertex] == u_max) );

		incomingEdgesSum.end();
	}
}
*/



kMST_ILP::~kMST_ILP()
{
	// free global CPLEX resources
	cplex.end();
	model.end();
	env.end();
}
