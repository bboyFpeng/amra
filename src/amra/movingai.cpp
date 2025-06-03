// project includes
#include <amra/movingai.hpp>
#include <amra/helpers.hpp>
#include <amra/constants.hpp>

// system includes

// standard includes
#include <fstream> //标准库中的文件流操作，文件读写
#include <cassert>//标准库中的断言操作，断言检查
#include <cstring>//标准库中的字符串操作，
#include <iomanip>//标准库中的格式化头文件，负责格式化输出

namespace AMRA  //定义了一个名为AMRA的命名空间，将后续代码封装在这个空间，避免命名冲突
{

MovingAI::MovingAI(const std::string& fname)//类构造函数，接受一个文件名为fname作为参数
:
m_fname(fname),//初始化列表中，将m_fname初始化为fname，m_rng初始化为一个随机数生成器。
m_rng(m_dev()) //调用了readFile()函数来读取文件内容，并定义了一个均匀分布的随机数生成器m_distD，范围是0.0到1.0。
{
	readFile();
	m_distD = std::uniform_real_distribution<double>(0.0, 1.0);
};

MovingAI::~MovingAI()//MovingAI类的析构函数，用于释放动态分配的内存m_map
{
	free(m_map);
}

void MovingAI::GetRandomState(int& d1, int& d2)
{
	while (true)
	{
		d1 = (int)std::round(m_distD(m_rng) * (m_h - 1));
		d2 = (int)std::round(m_distD(m_rng) * (m_w - 1));
		//std；round是将浮点数四舍五入为最接近的整数。
		if (NUM_RES == 2)
		{
			if ((d1 % MIDRES_MULT != 0 || d2 % MIDRES_MULT != 0)) {
				continue;
			}
		}
		if (NUM_RES == 3)
		{
			if ((d1 % LOWRES_MULT != 0 || d2 % LOWRES_MULT != 0)) {
				continue;
			}
		}

		if (IsTraversible(d1, d2)) {
			break;
		}
	}
}

void MovingAI::SavePath(    //保存路径到文件夹的地方
	const std::vector<MapState>& solpath,
	int iter)
{
	std::string filename(__FILE__);
	auto found = filename.find_last_of("/\\");
	filename = filename.substr(0, found + 1) + "../../dat/solutions/";//表示向上两级回退目录

	found = m_fname.find_last_of("/\\");
	filename += m_fname.substr(found + 1);//提取文件名拼接

	std::string pathfile(filename);
	pathfile.insert(pathfile.find_last_of('.'), "_");
	pathfile.insert(pathfile.find_last_of('.'), "path");

	if (iter >= 0)
	{
		std::stringstream ss;
		ss << std::setw(4) << std::setfill('0') << iter << '_';
		std::string s = ss.str();

		pathfile.insert(pathfile.find_last_of('/')+1, s);

		reset(ss);
	}

	std::ofstream OUT_PATH;
	OUT_PATH.open(pathfile, std::ofstream::out);
	for (const auto& s: solpath) {
		OUT_PATH << s;
	}
	OUT_PATH.close();
}

void MovingAI::SaveExpansions(
	int iter, double w1, double w2,
	const EXPANDS_t& expansions)
{
	std::string filename(__FILE__), expfile;
	auto found = filename.find_last_of("/\\");//找到最后一个斜杠的位置
	filename = filename.substr(0, found + 1) + "../../dat/expansions/";

	std::stringstream ss;//通过 << 写入数据，通过 ss.str() 获取结果字符串。常用于字符串拼接、类型转换
	ss << std::setw(4) << std::setfill('0') << iter << '_';
	ss << w1 << '_';
	ss << w2;
	std::string s = ss.str();

	filename += s;//将拼接好的字符串
	reset(ss);

	MAP_t expmap;
	expmap = (MAP_t)calloc(m_h * m_w, sizeof(decltype(*expmap)));
	//创建一个与原始地图相同大小的临时地图，分配并初始化为0，表示所有单元格初始为未扩展状态。
	for (const auto& q: expansions)//expansions 存储了算法在不同阶段扩展的节点集合，每个 q 对应一个时间步或迭代，包含该时刻扩展的所有节点。
	{
		std::memcpy(expmap, m_map, m_h * m_w * sizeof(decltype(*expmap)));
		//每次处理新阶段重置地图位院士状态，标记扩展节点，
		for (const auto& s: q.second) {
			expmap[GETMAPINDEX(s->coord.at(0), s->coord.at(1), m_h, m_w)] = MOVINGAI_DICT.find('E')->second;
			if (COSTMAP) {
				expmap[GETMAPINDEX(s->coord.at(0), s->coord.at(1), m_h, m_w)] *= 10;
			}//若使用成本地图，将标记值乘以10以保留成本信息。
		}

		expfile = filename;
		ss << std::setw(4) << std::setfill('0') << q.first << '_';
		found = expfile.find_last_of("/\\");
		expfile.insert(found+1+4+1, ss.str());
		reset(ss);

		std::ofstream EXP_MAP;
		EXP_MAP.open(expfile, std::ofstream::out);
		for (int r = 0; r < m_h; ++r)
		{
			for (int c = 0; c < m_w; ++c)
			{
				EXP_MAP << expmap[GETMAPINDEX(r, c, m_h, m_w)];

				if (c < m_w - 1) {
					EXP_MAP << ',';
				}
			}

			if (r < m_h - 1) {
				EXP_MAP << '\n';
			}
		}

		EXP_MAP.close();
	}

	free(expmap);
}

bool MovingAI::IsValid(const int& dim1, const int& dim2) const
{
	return (dim1 >= 0 && dim1 < m_h) && (dim2 >= 0 && dim2 < m_w);
}

bool MovingAI::IsTraversible(const int& dim1, const int& dim2) const
{
	if (!IsValid(dim1, dim2)) {
		return false;
	}
	return m_map[GETMAPINDEX(dim1, dim2, m_h, m_w)] > 0;
	//获取dim1和dim2坐标的索引值
}

int MovingAI::CellType(const int& dim1, const int& dim2) const
{
	if (!IsValid(dim1, dim2)) {
		return -99;
	}
	return m_map[GETMAPINDEX(dim1, dim2, m_h, m_w)];
}  //用于获取地图上指定坐标 (dim1, dim2) 处的单元格类型值。这个值代表了该位置的地形属性

int MovingAI::CellType(const int& dim1, const int& dim2, char& c) const
{
	c = '!';
	if (!IsValid(dim1, dim2)) {
		return -99;
	}

	int val = m_map[GETMAPINDEX(dim1, dim2, m_h, m_w)];
	for (auto itr = MOVINGAI_DICT.begin(); itr != MOVINGAI_DICT.end(); ++itr)
	{
		if (itr->second == val) {
			c = itr->first;
		}
	}
	if (c == '!') {
		c = char(val);
	}

	return val;
}

void MovingAI::readFile()
{
	std::ifstream FILE(m_fname);//打开文件
	std::string line, word, temp;
	std::stringstream ss;

	std::getline(FILE, line);
	assert(line.compare("type octile") == 0);//asset确保文件格式正确，否则程序崩溃

	// read height/width
	std::getline(FILE, line);
	reset(ss); //重置并清空字符串流，
	ss.str(line);
	std::getline(ss, word, ' ');//若 ss 内容是 "height 32"，则 word 变为 "height"，ss 剩余内容为 "32"。
	if (word.compare("height") == 0)
	{
		std::getline(ss, word, ' ');
		m_h = std::stoi(word);
	}
	else if (word.compare("width") == 0)
	{
		std::getline(ss, word, ' ');
		m_w = std::stoi(word);
	}

	// read width/height
	std::getline(FILE, line);
	reset(ss);
	ss.str(line);
	std::getline(ss, word, ' ');
	if (word.compare("height") == 0)
	{
		std::getline(ss, word, ' ');
		m_h = std::stoi(word);
	}
	else if (word.compare("width") == 0)
	{
		std::getline(ss, word, ' ');
		m_w = std::stoi(word);
	}

	std::getline(FILE, line);
	assert(line.compare("map") == 0);

	m_map = (MAP_t)calloc(m_h * m_w, sizeof(decltype(*m_map)));
	//使用 calloc 函数为 m_map 分配内存,m_h * m_w要分配的元素个数，izeof(decltype(*m_map))表示每个元素的大小，decltype(*m_map) 用于获取 m_map 所指向的元素类型
	for (int r = 0; r < m_h; ++r)
	{
		std::getline(FILE, line);//文件流 FILE 中读取一行文本，并将其存储到 line 中。每次循环读取一行地图数据，用于后续处理
		for (int c = 0; c < m_w; ++c)//遍历当前行的每一列
		{
			auto itr = MOVINGAI_DICT.find(line[c]);
			if (itr != MOVINGAI_DICT.end()) {
				m_map[GETMAPINDEX(r, c, m_h, m_w)] = MOVINGAI_DICT.find(line[c])->second;
			}
			else {
				int val = int(line[c]) - int('a') + 1;
				m_map[GETMAPINDEX(r, c, m_h, m_w)] = val * 10;
			}
		}
	}
}

}  // namespace AMRA
