#include<vector>
#include<algorithm>
#include<fstream>
#include<iostream>
#include<string>
#include<iterator>
#include<sstream>
#include<cmath>
#include<unordered_map>
#include<queue>

using namespace std;

#define REP(i,n) for(int i=0;i<(int)(n);i++) 
#define dump(n) cout<<"# "<<#n<<"="<<(n)<<endl

const unordered_map<int, string> NAME_MAP{
	{0,"Head"}, { 1,"Neck"}, { 2,"Torso"}, {3,"LeftShoulder"}, { 4,"LeftElbow"},
	{ 5,"LeftHand"}, {6,"RightShoulder"}, { 7,"RightElbow"}, { 8,"RightHand"},
	{9,"LeftHip"}, { 10,"LeftKnee"}, { 11,"LeftFoot"}, {12,"RightHip"},
	{ 13,"RightKnee"}, { 14,"RightFoot"}};
// 4, 5, 7, 8は使わない
const vector<int> EXCLUDES = {4,5,7,8};
const int JOINT_NUM = NAME_MAP.size();

const unordered_map<int, string> BONE_MAP{
	{0, "Head_to_Neck"}, {1, "Neck_to_Torso"}, {2, "Neck_to_LShoulder"},
	{3, "Neck_to_RShoulder"}, {4, "Torso_to_LHip"}, {5, "Torso_to_RHip"},
	{6, "LHip_to_LKnee"}, {7, "LKnee_to_LFoot"}, {8, "RHip_to_RKnee"}, {9, "RKnee_to_RFoot"}};
const int BONE_NUM = BONE_MAP.size();
const string OUTPUT_DIR = "output/";

class Vector{
	public:
		double x,y,z,t;
		Vector() = default;
		Vector(const double x, const double y, const double z, const double t) :
			x(x), y(y), z(z), t(t) {}
		const double norm() const{ return sqrt(x*x + y*y + z*z); };
		Vector normalized() const{
			double n = norm();
			return Vector(x/n, y/n, z/n, t);
		}
		// コサイン類似度
		double operator& (const Vector& right) const;
		// cos (ベクトルの角度)
		double operator* (const Vector& right) const;
		// ベクトル引き算
		Vector operator- (const Vector& right) const;
		// 外積
		Vector operator&& (const Vector& right) const;
};

typedef vector<Vector> vV;
vV calc_trajectory(const vV &v);
vV calc_diff1(const vV &v);
vV calc_diff2(const vV &v);
vV moving_average(const int n, const vV& v);

class Joint{
	public:
		string name;
		vV sequence, trajectory;
		vV diff1, diff1_traj;
		vV diff2, diff2_traj;
		Joint() = default;
		Joint(const string name, const vV seq, const vV diff1, const vV diff2): name(name), sequence(seq), trajectory(calc_trajectory(seq)), diff1(diff1), diff1_traj(calc_trajectory(diff1)), diff2(diff2), diff2_traj(calc_trajectory(diff2)) {}
		void push_back(const double x, const double y, const double z, const double t){
			sequence.push_back(Vector(x, y, z, t));
		}
		Joint normalized() const;
		Joint operator- (const Joint& right) const;
		Joint operator&& (const Joint& right) const;
		void write_csv(const string filename = "") const;
}; 

class Result{
	public:
		string name;
		double before, after;
		vector<double> before_sims, after_sims;
		Joint master_joint, before_joint, after_joint;
		Result() = default;
		Result(const string name, const pair<double, vector<double> > before, const pair<double, vector<double> > after) : name(name), after(after.first), before(before.first), after_sims(after.second), before_sims(before.second) {}
		Result(const string& name, const Joint master, const Joint before, const Joint after, const bool is_velocity = false);
		void write_csv(const string filename = "") const;
		bool operator< (const Result& right) const{
			return (after - before) < (right.after - right.before);
		}
};

class Point{
	public:
		short x,y;
		Point() : x(0),y(0) {}
		Point(int x,int y) : x(x),y(y) {}
};

class Node{
	public:
		double cost;
		Point prev;
		Point cur;
		Node() :cost((double)1e30) {}
		Node(double cost,const Point& prev,const Point& cur) : cost(cost),prev(prev),cur(cur) {}
		bool operator >(const Node& r) const{ return cost > r.cost; }
};

vector<Joint> csv_to_joint(const string filename, const int filter_n);
vector<Joint> joint_to_bone(const vector<Joint>& joints);
vector<vector<string> >csv_reader(const string filename);
void csv_writer(const string filename, const vector<vector<string> > &data);
void output_vector(const string filename, vV& data);

template<class T>
vector<string> map_to_string(const vector<T>& src){
	vector<string> dest;
	transform(begin(src), end(src), back_inserter(dest), static_cast<string(*)(T)>(to_string)); // static_castよくわからん, 調べよう.
	return dest;
}

template<class T>
void concat(vector<T>& dest, const vector<T> & src){
	dest.insert(end(dest), begin(src), end(src));
}

template<class T1>
/* arr  ---> Input Array
   n      ---> Size of input array
   r      ---> Size of a combination to be printed
   index  ---> Current index in data[]
   data ---> Temporary array to store current combination
   i      ---> index of current element in arr[]     */
void combinationloop(const vector<T1>& arr, int r, int index, vector<T1>& data, int i, vector<vector<T1> >& result){
	// Current cobination is ready
	int n = arr.size();
    if (index == r){
		result.push_back(data);
		return;
    }
	// When no more elements are there to put in data[]
	if (i >= n)
		return;
	// current is included, put next at next location
	data[index] = arr[i];
	combinationloop(arr, r, index+1, data, i+1, result);
	// current is excluded, replace it with next (Note that
	// i+1 is passed, but index is not changed)
	combinationloop(arr, r, index, data, i+1, result);
}  

template<class T1>
vector<vector<T1> > combination(const vector<T1>& arr, int r){
	vector<vector<T1> > result;
	vector<T1> data(r);
	combinationloop(arr, r, 0, data, 0, result);
	return result;
}  

template<class T2>
pair<double, vector<double> > DPmatching(const vector<T2>& v1, const vector<T2>& v2){
	int m = v1.size(), n = v2.size(), x, y;
	vector<vector<Node> > path_matrix(m, vector<Node>(n));
	Node point(v1[0]&v2[0], Point(-1,-1), Point(0,0));
	path_matrix[0][0] = point;
	//ダイクストラ法
	priority_queue<Node, vector<Node>, greater<Node> > p_queue;
	p_queue.push(point);
	while(!p_queue.empty()){
		Node cur_node = p_queue.top();
		p_queue.pop();
		if(path_matrix[cur_node.cur.x][cur_node.cur.y].cost < cur_node.cost){
			continue;
		}
		if(cur_node.cur.x == m-1 && cur_node.cur.y == n-1){
			break;
		}
		static const short dir_x[] = {0,1,1}, dir_y[] = {1,1,0};
		for(int i = 0; i < 3; i++){
			int nx = cur_node.cur.x + dir_x[i];
			int ny = cur_node.cur.y + dir_y[i];
			double add_cost = v1[nx] & v2[ny];
			if(nx < m && ny < n && path_matrix[nx][ny].cost > cur_node.cost + add_cost){
				path_matrix[nx][ny].cost = cur_node.cost + add_cost;
				path_matrix[nx][ny].prev = cur_node.cur;
				p_queue.push(Node(path_matrix[nx][ny].cost,cur_node.cur,Point(nx,ny)));
			}
		}
	}
	vector<double> costs;
	x = m - 1;
	y = n - 1;
	double prev_cost = path_matrix[x][y].cost;
	while (x != -1) {
		Node p = path_matrix[x][y];
		double diff = prev_cost - p.cost;
		costs.push_back(diff);
		prev_cost = p.cost;
		x = p.prev.x;
		y = p.prev.y;
	}
	
	reverse(costs.begin(), costs.end());
	/*
	for(auto a : costs)
		cout << a << ",";
	cout << endl;
	*/
	return make_pair(path_matrix[m-1][n-1].cost / costs.size(), costs);
}  


