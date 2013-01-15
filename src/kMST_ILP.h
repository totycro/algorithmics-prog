#ifndef __K_MST_ILP__H__
#define __K_MST_ILP__H__

#include "Tools.h"
#include "Instance.h"
#include <ilcplex/ilocplex.h>

#include <iostream>

using namespace std;

ILOSTLBEGIN

class kMST_ILP
{

private:

	// input data
	Instance& instance;
	string model_type;
	int k;
	// number of edges and nodes including root node and root edges
	u_int m, n;

	IloEnv env;
	IloModel model;
	IloCplex cplex;

	IloBoolVarArray edges; // first half one direction, second half other direction

	IloIntVarArray flow_scf; // only used for scf (admittedly somewhat ugly, but this is no coding course :)
	vector<IloBoolVarArray> flow_mcf; // used for mcf (same Hack here)
	IloIntVarArray u; // only used for mtz

	int nodes; //branch an bound nodes
	double objectiveValue; // cost
	double cpuTime; // time needed

	void modelSCF();
	void modelMCF();
	void modelMTZ();


public:

	kMST_ILP( Instance& _instance, string _model_type, int _k );
	~kMST_ILP();
	void solve();
	int getNodes();
	double getObjectiveValue();
	double getcpuTime();

private:

	void setCPLEXParameters();

	void addTreeConstraints();
	void addObjectiveFunction();

	void getOutgoingEdgeIds(vector<u_int> & outgoingEdges, u_int vertex);
	void getIncomingEdgeIds(vector<u_int> & incomingEdgeIds, u_int vertex);
	void getVertexEdgeIds(vector<u_int> & incomingEdgeIds, u_int vertex, bool outgoing); // internal


};
// kMST_ILP

#endif //__K_MST_ILP__H__
