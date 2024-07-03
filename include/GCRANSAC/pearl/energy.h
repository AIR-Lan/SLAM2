

#ifndef __ENERGY_H__
#define __ENERGY_H__

#include <assert.h>
#include "GCRANSAC/pearl/graph.h"

template <typename captype, typename tcaptype, typename flowtype>
class Energy : public Graph<captype, tcaptype, flowtype>
{
	typedef Graph<captype, tcaptype, flowtype> GraphT;

public:
	typedef typename GraphT::node_id Var;

	/* Types of energy values.
	   Value is a type of a value in a single term
	   TotalValue is a type of a value of the total energy.
	   By default Value = short, TotalValue = int.
	   To change it, change the corresponding types in graph.h */
	typedef captype Value;
	typedef flowtype TotalValue;

	/* interface functions */

	/* Constructor. Optional argument is the pointer to the
	   function which will be called if an error occurs;
	   an error message is passed to this function. If this
	   argument is omitted, exit(1) will be called. */
	Energy(int var_num_max, int edge_num_max, void (*err_function)(const char *) = NULL);

	/* Destructor */
	~Energy();

	/* Adds a new binary variable */
	Var add_variable(int num = 1);

	/* Adds a constant E to the energy function */
	void add_constant(Value E);

	/* Adds a new term E(x) of one binary variable
	   to the energy function, where
		   E(0) = E0, E(1) = E1
	   E0 and E1 can be arbitrary */
	void add_term1(Var x,
				   Value E0, Value E1);

	/* Adds a new term E(x,y) of two binary variables
	   to the energy function, where
		   E(0,0) = E00, E(0,1) = E01
		   E(1,0) = E10, E(1,1) = E11
	   The term must be regular, i.e. E00 + E11 <= E01 + E10 */
	void add_term2(Var x, Var y,
				   Value E00, Value E01,
				   Value E10, Value E11);

	/* Adds a new term E(x,y,z) of three binary variables
	   to the energy function, where
		   E(0,0,0) = E000, E(0,0,1) = E001
		   E(0,1,0) = E010, E(0,1,1) = E011
		   E(1,0,0) = E100, E(1,0,1) = E101
		   E(1,1,0) = E110, E(1,1,1) = E111
	   The term must be regular. It means that if one
	   of the variables is fixed (for example, y=1), then
	   the resulting function of two variables must be regular.
	   Since there are 6 ways to fix one variable
	   (3 variables times 2 binary values - 0 and 1),
	   this is equivalent to 6 inequalities */
	void add_term3(Var x, Var y, Var z,
				   Value E000, Value E001,
				   Value E010, Value E011,
				   Value E100, Value E101,
				   Value E110, Value E111);

	/* After the energy function has been constructed,
	   call this function to minimize it.
	   Returns the minimum of the function */
	TotalValue minimize();

	/* After 'minimize' has been called, this function
	   can be used to determine the value of variable 'x'
	   in the optimal solution.
	   Returns either 0 or 1 */
	int get_var(Var x);

	/***********************************************************************/
	/***********************************************************************/
	/***********************************************************************/

private:
	/* internal variables and functions */

	TotalValue Econst;
	void (*error_function)(const char *); /* this function is called if a error occurs,
											with a corresponding error message
											(or exit(1) is called if it's NULL) */
};

/***********************************************************************/
/************************  Implementation ******************************/
/***********************************************************************/

template <typename captype, typename tcaptype, typename flowtype>
inline Energy<captype, tcaptype, flowtype>::Energy(int var_num_max, int edge_num_max, void (*err_function)(const char *)) : Graph<captype, tcaptype, flowtype>(var_num_max, edge_num_max, err_function)
{
	Econst = 0;
	error_function = err_function;
}

template <typename captype, typename tcaptype, typename flowtype>
inline Energy<captype, tcaptype, flowtype>::~Energy() {}

template <typename captype, typename tcaptype, typename flowtype>
inline typename Energy<captype, tcaptype, flowtype>::Var Energy<captype, tcaptype, flowtype>::add_variable(int num)
{
	return GraphT::add_node(num);
}

template <typename captype, typename tcaptype, typename flowtype>
inline void Energy<captype, tcaptype, flowtype>::add_constant(Value A) { Econst += A; }

template <typename captype, typename tcaptype, typename flowtype>
inline void Energy<captype, tcaptype, flowtype>::add_term1(Var x,
														   Value A, Value B)
{
	this->add_tweights(x, B, A);
}

template <typename captype, typename tcaptype, typename flowtype>
inline void Energy<captype, tcaptype, flowtype>::add_term2(Var x, Var y,
														   Value A, Value B,
														   Value C, Value D)
{
	/*
	   E = A A  +  0   B-A
		   D D     C-D 0
	   Add edges for the first term
	*/
	this->add_tweights(x, D, A);
	B -= A;
	C -= D;

	/* now need to represent
	   0 B
	   C 0
	*/

//	assert(B + C >= 0); /* check regularity */
	if (B < 0)
	{
		/* Write it as
		   B B  +  -B 0  +  0   0
		   0 0     -B 0     B+C 0
		*/
		this->add_tweights(x, 0, B);	/* first term */
		this->add_tweights(y, 0, -B);	/* second term */
		this->add_edge(x, y, 0, B + C); /* third term */
	}
	else if (C < 0)
	{
		/* Write it as
		   -C -C  +  C 0  +  0 B+C
			0  0     C 0     0 0
		*/
		this->add_tweights(x, 0, -C);	/* first term */
		this->add_tweights(y, 0, C);	/* second term */
		this->add_edge(x, y, B + C, 0); /* third term */
	}
	else /* B >= 0, C >= 0 */
	{
		this->add_edge(x, y, B, C);
	}
}

template <typename captype, typename tcaptype, typename flowtype>
inline void Energy<captype, tcaptype, flowtype>::add_term3(Var x, Var y, Var z,
														   Value E000, Value E001,
														   Value E010, Value E011,
														   Value E100, Value E101,
														   Value E110, Value E111)
{
	register Value pi = (E000 + E011 + E101 + E110) - (E100 + E010 + E001 + E111);
	register Value delta;
	register Var u;

	if (pi >= 0)
	{
		Econst += E111 - (E011 + E101 + E110);

		add_tweights(x, E101, E001);
		add_tweights(y, E110, E100);
		add_tweights(z, E011, E010);

		delta = (E010 + E001) - (E000 + E011); /* -pi(E[x=0]) */
		assert(delta >= 0);					   /* check regularity */
		add_edge(y, z, delta, 0);

		delta = (E100 + E001) - (E000 + E101); /* -pi(E[y=0]) */
		assert(delta >= 0);					   /* check regularity */
		add_edge(z, x, delta, 0);

		delta = (E100 + E010) - (E000 + E110); /* -pi(E[z=0]) */
		assert(delta >= 0);					   /* check regularity */
		add_edge(x, y, delta, 0);

		if (pi > 0)
		{
			u = add_variable();
			add_edge(x, u, pi, 0);
			add_edge(y, u, pi, 0);
			add_edge(z, u, pi, 0);
			add_tweights(u, 0, pi);
		}
	}
	else
	{
		Econst += E000 - (E100 + E010 + E001);

		add_tweights(x, E110, E010);
		add_tweights(y, E011, E001);
		add_tweights(z, E101, E100);

		delta = (E110 + E101) - (E100 + E111); /* -pi(E[x=1]) */
		assert(delta >= 0);					   /* check regularity */
		add_edge(z, y, delta, 0);

		delta = (E110 + E011) - (E010 + E111); /* -pi(E[y=1]) */
		assert(delta >= 0);					   /* check regularity */
		add_edge(x, z, delta, 0);

		delta = (E101 + E011) - (E001 + E111); /* -pi(E[z=1]) */
		assert(delta >= 0);					   /* check regularity */
		add_edge(y, x, delta, 0);

		u = add_variable();
		add_edge(u, x, -pi, 0);
		add_edge(u, y, -pi, 0);
		add_edge(u, z, -pi, 0);
		this->add_tweights(u, -pi, 0);
	}
}

template <typename captype, typename tcaptype, typename flowtype>
inline typename Energy<captype, tcaptype, flowtype>::TotalValue Energy<captype, tcaptype, flowtype>::minimize()
{
	return Econst + GraphT::maxflow();
}

template <typename captype, typename tcaptype, typename flowtype>
inline int Energy<captype, tcaptype, flowtype>::get_var(Var x) { return static_cast<int>(this->what_segment(x)); }

#endif
