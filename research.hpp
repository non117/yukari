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

// TODO : フィルタ, びぶん, ベクトル角度, csv export, グラフ描画, データの修正

const unordered_map<int, string> NAME_MAP{
	{0,"Head"}, { 1,"Neck"}, { 2,"Torso"}, {3,"LeftShoulder"}, { 4,"LeftElbow"},
	{ 5,"LeftHand"}, {6,"RightShoulder"}, { 7,"RightElbow"}, { 8,"RightHand"},
	{9,"LeftHip"}, { 10,"LeftKnee"}, { 11,"LeftFoot"}, {12,"RightHip"},
	{ 13,"RightKnee"}, { 14,"RightFoot"}};
const int JOINT_NUM = NAME_MAP.size();
/*
const unordered_map<int, string> NAME_MAP2{
	{0,"Torso-Neck"},{1,"Neck-LShoulder"},{2,"LShoulder-LElbow"},{3,"LElbow-LHand"},
	{4,"Neck-RShoulder"},{5,"RShoulder-RElbow"},{6,"RElbow-RHand"},{7,"Neck-Head"},
	{8,"Torso-LHip"},{9,"LHip-LKnee"},{10,"LKnee-LFoot"},{11,"Torso-RHip"},
	{12,"RHip-RKnee"},{13,"RKnee-RFoot"}};
const int JOINT_NUM2 = NAME_MAP2.size();
*/

class Vector{
	public:
		double x,y,z,t;
		Vector() = default;
		Vector(const double x, const double y, const double z, const double t);
		const double norm() const;
		double operator* (const Vector& right) const;
};
class Joint{
	public:
		string name;
		vector<Vector> sequence;
		Joint() = default;
		void push_back(const double x, const double y, const double z, const double t);
}; 

struct Point{
    short x,y;
    Point() : x(0),y(0) {}
    Point(int x,int y) : x(x),y(y) {}
};

struct Node{
    double cost;
    Point prev;
    Point cur;
    Node() :cost((double)1e30) {}
    Node(double cost,const Point& prev,const Point& cur) : cost(cost),prev(prev),cur(cur) {}
    bool operator >(const Node& r) const{ return cost > r.cost; }
};

vector<Joint> csv_to_joint(const string filename);
vector<vector<string> >csv_reader(const string filename);
void csv_writer(const string filename, const vector<vector<string> > &data);
vector<Vector> convert_to_trajectory(const vector<Vector>& v);
double DPmatching(const vector<Vector>& v1, const vector<Vector>& v2);
double DPmatching2(const vector<Vector>& v1, const vector<Vector>& v2); 


