#ifndef __TOOLS__CPP__
#define __TOOLS__CPP__

#include "Tools.h"

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
	ss << "edge(" << setw(2);
	if (!direction) {
		ss << edge.v1 << " to " << edge.v2 ;
	} else {
		ss << edge.v2 << " to " << edge.v1 ;
	}
	ss << setw(0) << ", w:" << edge.weight << ")";
	return ss.str();
}


double Tools::CPUtime()
{
	tms t;
	times( &t );
	double ct = sysconf( _SC_CLK_TCK );
	return t.tms_utime / ct;
}

#endif // __TOOLS__CPP__
