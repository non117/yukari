#include<vector>
#include<fstream>
#include<iostream>
#include<string>
#include<sstream>
#include<cmath>
#include<unordered_map>
#include<queue>

using namespace std;

#define REP(i,n) for(int i=0;i<(int)(n);i++) 
#define dump(n) cout<<"# "<<#n<<"="<<(n)<<endl

// TODO : データの修正

const unordered_map<int, string> NAME_MAP{
	{0,"Head"}, { 1,"Neck"}, { 2,"Torso"}, {3,"LeftShoulder"}, { 4,"LeftElbow"},
	{ 5,"LeftHand"}, {6,"RightShoulder"}, { 7,"RightElbow"}, { 8,"RightHand"},
	{9,"LeftHip"}, { 10,"LeftKnee"}, { 11,"LeftFoot"}, {12,"RightHip"},
	{ 13,"RightKnee"}, { 14,"RightFoot"}};
const int JOINT_NUM = NAME_MAP.size();

const unordered_map<int, string> BONE_MAP{
	{0, "Head_to_Neck"}, {1, "Neck_to_Torso"}, {2, "Neck_to_LShoulder"},
	{3, "Neck_to_RShoulder"}, {4, "Torso_to_LHip"}, {5, "Torso_to_RHip"},
	{6, "LHip_to_LKnee"}, {7, "LKnee_to_LFoot"}, {8, "RHip_to_RKnee"}, {9, "RKnee_to_RFoot"}};
const int BONE_NUM = BONE_MAP.size();

class Vector{
	public:
		double x,y,z,t;
		Vector() = default;
		Vector(const double x, const double y, const double z, const double t) :
			x(x), y(y), z(z), t(t) {}
		const double norm() const{ return sqrt(x*x + y*y + z*z); };
		// コサイン類似度
		double operator& (const Vector& right) const;
		// cos (ベクトルの角度)
		double operator* (const Vector& right) const;
		// ベクトル引き算
		Vector operator- (const Vector& right) const;
};

typedef vector<Vector> vV;
vV calc_trajectory(vV &v);
vV calc_diff1(vV &v);
vV calc_diff2(vV &v); 
vV moving_average(const int n, const vV& v);

/*class vV2 : public vector<Vector>{
	vV2 calc_trajectory();
	vV2 calc_diff1();
	vV2 calc_diff2();
	vector<vector<string> > convert_to_string_matrix();
};*/

class Joint{
	public:
		string name;
		vV sequence;
		vV trajectory;
		vV diff1;
		vV diff1_traj;
		vV diff2;
		vV diff2_traj;
		Joint() = default;
		void push_back(const double x, const double y, const double z, const double t){
			sequence.push_back(Vector(x, y, z, t));
		}
		Joint operator- (const Joint& right) const;
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
double DPmatching(const vector<T2>& v1, const vector<T2>& v2){
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
	return path_matrix[m-1][n-1].cost / costs.size();
}  


