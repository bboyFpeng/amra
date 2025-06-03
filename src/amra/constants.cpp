#include <amra/constants.hpp>

const int AMRA::MAX_PLANNING_TIME_MS = 30000;//路径规划的时间限制
const int AMRA::COST_MULT = 100; //成本乘数，用于将成本计算转换为证书，便于后续处理，这里是100.
const std::map<char, int> AMRA::MOVINGAI_DICT = {    //movingai地图格式的字符到整数的映射
	{'.', 1},
	{'G', 1},
	{'@', -1},
	{'O', -1},
	{'T', 0},
	{'S', 1},
	{'W', 2}, // water is only traversible from water
	{'(', 1000}, // start
	{'*', 1001}, // path
	{')', 1002}, // goal
	{'E', 1003}, // expanded state
};

// discretisation resolutions for mid- and low-level grids
// high-level grids assume a 1x1 discretisation
const int AMRA::MIDRES_MULT = 3;
const int AMRA::LOWRES_MULT = 9;
// number of resolutions to use in the search (>= 1, <= 3)
const int AMRA::NUM_RES = 2;//搜索算法中实际使用的分辨率数量
// 4-connected or 8-connected grid
const int AMRA::GRID = 4;
// set true if using maps with non-uniform cell costs
const bool AMRA::COSTMAP = false;//表示是否使用具有非均匀单元格成本的地图

// UAV experiment parameters
const double AMRA::TURNING_RADIUS = 20.0;
const double AMRA::MAX_VEL = 8.0; //最大速度
const int AMRA::WP_TIME = 50; // milliseconds 航点时间

// use dubins or dijkstra heuristics? 是否使用对应的算法
const bool AMRA::DUBINS = false;
const bool AMRA::DIJKSTRA = false;

// run successive search iterations from scratch?
// (with no reuse of previous search effort)  是否从头开始执行连续的搜索迭代
const bool AMRA::SUCCESSIVE = false;
