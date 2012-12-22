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

	void modelSCF();
	void modelMCF();
	void modelMTZ();

public:

	kMST_ILP( Instance& _instance, string _model_type, int _k );
	~kMST_ILP();
	void solve();

private:

	void setCPLEXParameters();

	void addTreeConstraints();
	void addObjectiveFunction();


};
// kMST_ILP

#endif //__K_MST_ILP__H__
