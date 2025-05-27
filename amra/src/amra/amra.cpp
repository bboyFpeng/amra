// project includes
#include <amra/amra.hpp>
#include <amra/constants.hpp>
#include <amra/types.hpp>
#include <amra/heuristic.hpp>
#include <amra/helpers.hpp>

// system includes
#include <smpl/console/console.h>

// standard includes
#include <algorithm>

namespace AMRA
{

AMRAStar::AMRAStar(
	Environment* space,
	const std::vector<std::shared_ptr<Heuristic>>& heurs,
	const std::vector<std::pair<Resolution::Level, int> >& heurs_map,
	int heur_count, int res_count)
:
m_space(space),
m_call_number(0),
m_heur_count(heur_count),
m_res_count(res_count),
m_w1_i(10.0), m_w2_i(20.0),
m_w1_f(1.0), m_w2_f(1.0),
m_w1_delta(0.5), m_w2_delta(0.5),
m_start_id(-1),
m_goal_id(-1)
{
	// Set default max planing time
	m_time_limit = double(MAX_PLANNING_TIME_MS / 1000.0); // seconds

	if (COSTMAP) {
		m_w1_i = 500.0;
		m_w2_i = 500.0;

		m_w1_delta = 0.33;
		m_w2_delta = 0.33;
	}

	m_w1 = m_w1_i;
	m_w2 = m_w2_i;

	m_heurs = heurs;
	m_heurs_map = heurs_map;
	m_open = new OpenList[m_heurs_map.size()];  // inadmissible(s) + anchor
	m_expands = new int[m_heurs_map.size()];

	m_offset = std::numeric_limits<int>::max();
	for (const auto& pair: m_heurs_map)
	{
		int hres = static_cast<int>(pair.first);
		if (hres > 0 && hres < m_offset) {
			m_offset = hres;
		}
	}

	m_initial_t = 0.0;
	m_final_t = 0.0;
	m_initial_c = -1;
	m_final_c = -1;
	m_total_e = -1;
}

AMRAStar::~AMRAStar()
{
	reset();
}

int AMRAStar::set_start(int start_id)
{
	m_start_id = start_id;
	m_start = get_state(m_start_id);
	return m_start_id;
}

int AMRAStar::set_goal(int goal_id)
{
	m_goal_id = goal_id;
	m_goal = get_state(m_goal_id);
	return m_goal_id;
}

int AMRAStar::get_n_expands() const
{
	int expansions = 0;
	for (int i = 0; i < num_heuristics(); ++i) {
		expansions += m_expands[i];
	}
	return expansions;
}

std::string AMRAStar::get_expands_str() const
{
	std::string expansions;
	for (int i = 0; i < num_heuristics(); ++i) {
		expansions += std::to_string(m_expands[i]);
		if (i < num_heuristics() - 1) {
			expansions += ", ";
		}
	}
	return expansions;
}

void AMRAStar::reset()
{
	// Clear OPEN lists
	for (int i = 0; i < num_heuristics(); ++i) {
		if (!m_open[i].empty()) {
			m_open[i].clear();
		}
	}

	// free states
	for (size_t i = 0; i < m_states.size(); ++i)
	{
		if (m_states[i] != nullptr)
		{
			free(m_states[i]);
			// m_states[i] = nullptr;
		}
	}

	// Clear state table
	m_states.clear();
	// m_states.shrink_to_fit();

	m_start_id = -1;
	m_goal_id = -1;
	m_start = nullptr;
	m_goal = nullptr;
}

// Get the search state corresponding to a graph state, creating a new state if
// one has not been created yet.
AMRAState* AMRAStar::get_state(int state_id)
//用于获取或创建状态节点的核心函数。输入为状态ID输出为返回对应的AMRAState指针
{
	assert(state_id >= 0);

	if (m_states.size() <= state_id)//状态不存在，需要创建
	{
		//动态内存分配，基础大小：sizeof(AMRAState)，额外空间：为每个额外的启发式函数分配 HeapData
		size_t state_size =
			sizeof(AMRAState) +
			sizeof(AMRAState::HeapData) * (num_heuristics() - 1);
			//  +
			// sizeof(bool) * (m_res_count - 1);
		AMRAState* s = (AMRAState*)malloc(state_size);

		// Use placement new(s) to construct allocated memory，内存初始化
		new (s) AMRAState;
		// for (int i = 0; i < m_res_count-1; ++i) {
		// 	new (&s->closed_in_res[1 + i]) bool;
		// }
		for (int i = 0; i < num_heuristics() - 1; ++i) {
			new (&s->od[1 + i]) AMRAState::HeapData;
		}

		// assert(state_id == m_states.size());

		init_state(s, state_id);//自定义初始化函数
		m_states.push_back(s);//将新状态加入容器

		return s;
	}

	return m_states[state_id];//直接返回已存在的状态
}

void AMRAStar::init_state(AMRAState *state, int state_id)
{
	state->call_number = 0; // not initialized for any iteration
	state->state_id = state_id;
	for (int i = 0; i < num_heuristics(); ++i) {
		state->od[i].me = state;
	}
}

// Lazily (re)initialize a search state.
void AMRAStar::reinit_state(AMRAState *state)
{
	if (state->call_number != m_call_number) {
		state->call_number = m_call_number;
		state->g = std::numeric_limits<unsigned int>::max();
		state->res = m_space->GetResLevel(state->state_id);
		state->bp = nullptr;

		for (int i = 0; i < num_heuristics(); ++i) {
			state->od[i].h = compute_heuristic(state->state_id, i);
			state->od[i].f = std::numeric_limits<unsigned int>::max();
		}

		state->closed_in_anc = false;
		for (int i = 0; i < m_res_count; ++i) {
			state->closed_in_res[i] = false;
		}
	}
}

int AMRAStar::replan(
	std::vector<int>* solution_path,//路径序列
	std::vector<int>* action_ids,//记录动作指令
	int* solution_cost)//存储路径质量
{
	//处理起点与终点相同的情况
	if (is_goal(m_start_id))//判断起点是否为终点
	{
		// m_logger->LogMsg("Start is goal!", LogLevel::WARN);
		solution_path->push_back(m_start_id);//将起点ID添加到路径中，此时路径长度为1，即 [start_id]
		solution_cost = 0;//代价计算
		return 1;
	}

	m_w1 = m_w1_i;  //初始化权重w1为初始值
	m_w2 = m_w2_i;	//初始化权重w2
	m_w1_solve = -1.0; //标记为尚未找到解
	m_w2_solve = -1.0;

	//重置所有启发式函数的扩展节点数，num_heuristics()为返回启发式函数的数量
	for (int i = 0; i < num_heuristics(); ++i) {
		m_expands[i] = 0;
	}

	m_call_number++; // 增加搜索调用次数计数
	reinit_state(m_goal);	 // 重置目标状态
	reinit_state(m_start);	// 重置起始状态
	m_start->g = 0;		  // 设置起始状态的g值为0

	// clear all OPEN lists	清空所有启发式函数对应的OPEN列表
	for (int i = 0; i < num_heuristics(); ++i) {
		m_open[i].clear();
	}

	m_incons.clear();// 清空不一致列表,不一致列表为存储 g 值被更新但尚未重新评估的节点
	m_incons.push_back(m_start);// 将 《起点》 加入不一致列表

	m_search_time = 0.0;// 重置搜索时间统计
	m_iter = 0;// 重置迭代次数


	//终止条件，达到时间限制 (m_search_time >= m_time_limit)，达到最小权重阈值 (m_w1 < m_w1_f || m_w2 < m_w2_f)
	while (m_search_time < m_time_limit && (m_w1 >= m_w1_f && m_w2 >= m_w2_f))
	{
		 // 1. 处理不一致列表中的节点
    	// 2. 重新计算各分辨率下的启发式值
    	// 3. 执行路径改进
    	// 4. 提取并记录当前最优路径
    	// 5. 调整权重，准备下一轮迭代

		for (auto* s : m_incons)//将不一致列表中的所有节点重新加入 OPEN 列表
		{
			s->od[0].f = compute_key(s, 0);//compute_key()：计算节点的优先级值 (f 值)
			s->closed_in_anc = false;
			insert_or_update(s, 0); //insert_or_update()：将节点插入或更新到 OPEN 列表
		}
		m_incons.clear();//清空不一致列表

		for (auto it = m_open[0].begin(); it != m_open[0].end(); ++it)
		//遍历最高分辨率OPEN列表，m_open[0]：最高分辨率（最精细）的 OPEN 列表
		{
			for (auto hidx = 1; hidx < num_heuristics(); hidx++)
			{
				//hidx：启发式函数索引（从 1 开始，0 为最高分辨率）
				// numerically greater resolutions are coarser
				if ((*it)->me->res >= m_heurs_map.at(hidx).first)
				//仅当节点分辨率不低于启发式要求时，应用该启发式
				//(*it)->me->res：当前节点的分辨率，m_heurs_map.at(hidx).first：第 hidx 个启发式函数要求的最小分辨率
				{
					(*it)->me->od[hidx].f = compute_key((*it)->me, hidx);
					//od[hidx]：对应 hidx 分辨率的 OpenData 结构，compute_key()：计算节点在 hidx 分辨率下的 f 值
					int hres = static_cast<int>(m_heurs_map.at(hidx).first);
					//closed_in_res记录节点在各分辨率下的关闭状态，hres - m_offset将绝对分辨率转换为数组索引
					(*it)->me->closed_in_res[hres - m_offset] = false;
					insert_or_update((*it)->me, hidx);//更新open表
				}
			}
		}
		reorder_open();//重新排序OPEN列表

		for (size_t i = 0; i < m_states.size(); ++i)
		{
			if (m_states[i] != nullptr)
			{
				m_states[i]->closed_in_anc = false; //// 重置祖先分辨率关闭标记
				for (int j = 0; j < m_res_count; ++j) {
					m_states[i]->closed_in_res[j] = false;  //重置各分辨率关闭标记
				}
			}
		}

		double search_start_time = GetTime(); // 记录搜索开始时间
		double search_time = 0.0;     // 本次搜索耗时
		int curr_exps = get_n_expands(); // 记录当前扩展节点数

		// 执行路径改进并返回是否找到解
		bool result = improve_path(search_start_time, search_time);

		m_search_time += search_time;		// 累计总搜索时间

		// 判断终止条件
		if(!result || m_search_time >= m_time_limit) {
			break;			// 未找到解或达到时间限制则退出
		}

		// 记录首次找到解的时间
		if (m_w1_solve < 0 || m_w2_solve < 0) {
			m_initial_t = m_search_time;
		}

		// 1. 提取路径和动作序列
		extract_path(*solution_path, *action_ids, *solution_cost);

		// 2. 记录日志信息
		SMPL_INFO("Solved with (%f, %f) | expansions = %s | time = %f | cost = %d", m_w1, m_w2, get_expands_str().c_str(), search_time, *solution_cost);
		
		// 3. 保存扩展节点数据
		if (curr_exps < get_n_expands()) {
			m_space->SaveExpansions(m_iter, m_w1, m_w2, *solution_path, *action_ids);
		}

		// 4. 判断是否达到最终权重
		if (m_w1 == m_w1_f && m_w2 == m_w2_f) {
			break;
		}

		// 5. 调整权重参数
		m_w1 = std::max(m_w1_f, m_w1 * m_w1_delta);
		m_w2 = std::max(m_w2_f, m_w2 * m_w2_delta);

		// 6. 迭代计数
		m_iter++;

		//SUCCESSIVE 模式处理
		if (SUCCESSIVE)
		{
			++m_call_number;

			reinit_state(m_goal);
			reinit_state(m_start);
			m_start->g = 0;

			// clear all OPEN lists
			for (int i = 0; i < num_heuristics(); ++i) {
				m_open[i].clear();
			}

			m_incons.clear();
			m_incons.push_back(m_start);
		}
	}

	if (m_w1_solve < 0 || m_w2_solve < 0)
	{
		solution_path->clear();
		*solution_cost = -1;
		return 0;
	}

	m_final_t = m_search_time;
	m_final_c = *solution_cost;
	m_total_e = get_n_expands();

	return 1;
}

bool AMRAStar::improve_path(	//返回值为是否找到路径
	const double& start_time,//搜索开始的时间点
	double& elapsed_time)		//输出参数，记录本次搜索花费的时间
{
	elapsed_time = 0.0;		//初始化耗时计时器为0
	while (!m_open[0].empty() &&
				m_open[0].min()->f < std::numeric_limits<unsigned int>::max())
	//循环条件：1.主OPEN 列表（索引 0）不为空，2.列表中最小的 f 值不是无穷大（表示还有可扩展的节点）
	{
		elapsed_time = GetTime() - start_time;
		//检查是否超过时间限制，计算当前搜索已用时间
		if (elapsed_time + m_search_time >= m_time_limit) {
			return false;
		}
		//总时间超过限制，返回失败。

		for (int i = 1; i < num_heuristics(); ++i)
		//遍历不同分辨率的启发式函数
		{
			if (m_open[0].empty()) {
				return false;		//如果OPEN表为空，返回失败
			}

			unsigned int f_check = m_w2 * m_open[0].min()->f;
			//计算 f_check：当前最优节点的 f 值乘以权重 m_w2
			if (m_goal->g <= f_check) {
				return true;   //如果目标节点的 g 值（实际代价）小于等于 f_check，说明已找到足够好的路径，返回成功
			}

			//决定从哪个列表扩展节点，
			//如果第 i 个 OPEN 列表不为空，且其最小 f 值小于等于 f_check
			//满足条件则从低分辨率扩展，不满足条件则从主列表扩展
			if (!m_open[i].empty() &&
					m_open[i].min()->f <= f_check)
			//从低分辨率列表扩展
			{
				AMRAState *s = m_open[i].min()->me;
				if (s->state_id == m_goal_id) {
					return true;
				}
				expand(s, i);
				++m_expands[i];
			}

			//从主列表扩展
			else
			{
				// expand from anchor
				AMRAState *s = m_open[0].min()->me;
				if (s->state_id == m_goal_id) {
					return true;
				}
				expand(s, 0);
				++m_expands[0];
			}
		}
	}
}

void AMRAStar::expand(AMRAState *s, int hidx)
{
	// close s in correct resolution
	// and remove from appropriate OPENs

	int hres_i = static_cast<int>(m_heurs_map.at(hidx).first);
	if (hidx == 0)
	{
		assert(!s->closed_in_anc);
		s->closed_in_anc = true;

		if (m_open[0].contains(&s->od[0])) {
			m_open[0].erase(&s->od[0]);
		}
	}
	else
	{
		assert(!s->closed_in_res[hres_i - m_offset]);
		s->closed_in_res[hres_i - m_offset] = true;
		if (m_open[hidx].contains(&s->od[hidx])) {
			m_open[hidx].erase(&s->od[hidx]);
		}

		for (int j = 1; j < num_heuristics(); ++j)
		{
			if (j != hidx && hres_i == static_cast<int>(m_heurs_map.at(j).first))
			{
				if (m_open[j].contains(&s->od[j])) {
					m_open[j].erase(&s->od[j]);
				}
			}
		}
	}

	std::vector<int> succ_ids;
	std::vector<unsigned int> costs;
	std::vector<int> action_ids;
	if (is_goal(s->state_id))
	{
		succ_ids.push_back(m_goal_id);
		costs.push_back(0);
		action_ids.push_back(-1);
	}
	else
	{
		m_space->GetSuccs(s->state_id,
						  static_cast<Resolution::Level>(hres_i),
						  &succ_ids,
						  &costs,
						  &action_ids);
	}

	for (size_t sidx = 0; sidx < succ_ids.size(); ++sidx)
	{
		unsigned int cost = costs[sidx];

		AMRAState *succ_state = get_state(succ_ids[sidx]);
		reinit_state(succ_state);

		unsigned int new_g = s->g + costs[sidx];
		if (new_g < succ_state->g)
		{
			succ_state->g = new_g;
			succ_state->bp = s;
			if (!action_ids.empty()) {
				succ_state->actionidx = action_ids[sidx];
			}
			if (succ_state->closed_in_anc) {
				m_incons.push_back(succ_state);
			}
			else
			{
				unsigned int f_0 = compute_key(succ_state, 0);
				succ_state->od[0].f = f_0;
				insert_or_update(succ_state, 0);

				for (int j = 1; j < num_heuristics(); ++j)
				{
					int hres_j = static_cast<int>(m_heurs_map.at(j).first);
					// if state resolution is coarser than queue resolution,
					// insert or update in queue
					// this assumes a high resolution grid coincides with
					// all grids coarser than it
					if (static_cast<int>(succ_state->res) >= hres_j)
					{
						if (!succ_state->closed_in_res[hres_j - m_offset])
						{
							unsigned int f_j = compute_key(succ_state, j);
							if (f_j <= m_w2 * f_0)
							{
								succ_state->od[j].f = f_j;
								insert_or_update(succ_state, j);
							}
						}
					}
				}
			}
		}
	}
}

bool AMRAStar::is_goal(int state_id)
{
	return m_space->IsGoal(state_id);
}

unsigned int AMRAStar::compute_heuristic(int state_id, int hidx)
{
	assert(num_heuristics() >= hidx);
	return m_heurs.at(m_heurs_map.at(hidx).second)->GetGoalHeuristic(state_id);
}

unsigned int AMRAStar::compute_key(AMRAState *state, int hidx)
{
	return state->g + m_w1 * state->od[hidx].h;
}

void AMRAStar::insert_or_update(AMRAState *state, int hidx)
{
	if (m_open[hidx].contains(&state->od[hidx])) {
		m_open[hidx].update(&state->od[hidx]);
	}
	else {
		m_open[hidx].push(&state->od[hidx]);
	}
}

void AMRAStar::reorder_open()
{
	for (auto hidx = 0; hidx < num_heuristics(); hidx++)
	{
		for (auto it = m_open[hidx].begin(); it != m_open[hidx].end(); ++it) {
			(*it)->f = compute_key((*it)->me, hidx);
		}
		m_open[hidx].make();
	}
}

void AMRAStar::extract_path(
	std::vector<int>& solution,
	std::vector<int>& action_ids,
	int& cost)
{
	if (m_w1_solve < 0 || m_w2_solve < 0) {
		m_initial_c = m_goal->g;
	}

	m_w1_solve = m_w1;
	m_w2_solve = m_w2;
	cost = m_goal->g;
	m_solution_cost = m_goal->g;

	solution.clear();
	action_ids.clear();

	// m_goal->state_id == m_goal_id == 0 should be true
	for (AMRAState *state = m_goal; state; state = state->bp) {
		solution.push_back(state->state_id);
		action_ids.push_back(state->actionidx);
	}
	std::reverse(solution.begin(), solution.end());
	std::reverse(action_ids.begin(), action_ids.end());
}

}  // namespace AMRA
