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
vector<vector<int> > combination(const vector<int>& arr, int r);
double DPmatching(const vV& v1, const vV& v2);


