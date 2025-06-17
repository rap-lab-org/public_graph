
/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/


#ifndef ZHONGQIANGREN_BASIC_SEARCH_DIJKSTRA_H_
#define ZHONGQIANGREN_BASIC_SEARCH_DIJKSTRA_H_

#include "search.hpp"
#include <set>

#define DEBUG_DIJKSTRA 0

namespace raplab{

/**
 * @brief
 * 
 * CAVEAT: The graph vertex ID should be within range [0,N], since std::vector is used as the underlying storage.
 * NOTE: This implementation assumes the entire graph is available.
 */
class Dijkstra : public GraphSearch
{
public:
	/**
	 *
	 */
	Dijkstra() ;
	/**
	 *
	 */
	virtual ~Dijkstra() ;
	/**
	 * @brief cdim specifies which cost dimension of the graph will be used for search.
	 */
	virtual std::vector<long> PathFinding(long vs, long vg, double time_limit = std::numeric_limits<double>::infinity(), short cdim = 0) override ;	
	/**
	 * @brief This must be called after calling PathFinding().
	 */
	virtual std::vector<double> GetSolutionCost() override ;
	/**
	 *
	 */
	virtual int ExhaustiveBackwards(long vg, double time_limit = std::numeric_limits<double>::infinity(), short cdim = 0) ;
	/**
	 *
	 */
	virtual int ExhaustiveForwards(long vs, double time_limit = std::numeric_limits<double>::infinity(), short cdim = 0) ;
	/**
	 *
	 */
	virtual std::vector<long> GetPath(long v, bool do_reverse=true) ;
	/**
	 * @brief Return a vector that stores the cost-to-go from all other vertices to/from the 
	 *   given vg/vs, depending on whether ExhaustiveBackwards/ExhaustiveForwards is called.
	 */
	virtual std::vector<double> GetDistAll() ;
	/**
	 * @brief Return the cost-to-go from v to/from the 
	 *   given vg/vs, depending on whether ExhaustiveBackwards/ExhaustiveForwards is called.
	 */
	virtual double GetDistValue(long v) ;
	/**
	 * @brief Similar to GetSolutionCost(), but must be called after ExhaustiveBackwards/ExhaustiveForwards.
	 */
	virtual std::vector<double> GetPathCost(long v) ;

protected:

	virtual int _search() ;

	// with this method, A* can be easily implemented by inheriting Dijkstra.
	virtual void _add_open(long v, double g) ;

	virtual void _init_more();

	// Graph* _graph; // inherited

	/////////

	// long _vs;
	// long _vg;
	
	short _cdim; // the selected cost dimenion to be searched.
	short _mode; // 0 = start-goal path finding, 1 = exhaustive backwards, 2 = exhaustive forwards.
	std::vector<long> _parent; // help reconstruct the path.
	std::vector<double> _v2d; // store the results.
	std::vector<std::vector<double>> _cvec; // the corresponding cost vector of the path to vd.
	std::set< std::pair<double, long> > _open;
	std::string _class_name = "Dijkstra";
};

} // end namespace zr

#endif  // ZHONGQIANGREN_BASIC_SEARCH_DIJKSTRA_H_
