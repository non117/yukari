#include "research.hpp"

double Vector::operator& (const Vector& right) const{
	double up = x * right.x + y * right.y + z * right.z;
	double down = norm() * right.norm();
	double res = up / down;
	if(res < 0)
		res = 0;
	return 1 - res;
}

double Vector::operator* (const Vector& right) const{
	double up = x * right.x + y * right.y + z * right.z;
	double down = norm() * right.norm();
	return up / down;
}  

Vector Vector::operator- (const Vector& right) const{
	Vector result = Vector(x - right.x, y - right.y, z - right.z, t);
	return result;
}

Vector Vector::operator&& (const Vector& right) const{
	Vector result = Vector(y*right.z - z*right.y, z*right.x - x*right.z, x*right.y - y*right.x, t);
	return result;
}

Joint Joint::operator- (const Joint& right) const{
	Joint result;
	int length = sequence.size();
	vV temp_sequence;
	for(int i=0;i<length;i++){
		temp_sequence.push_back(sequence[i] - right.sequence[i]);
	}
	result.sequence = temp_sequence;
	result.trajectory = calc_trajectory(temp_sequence);
	result.diff1 = calc_diff1(temp_sequence);
	result.diff1_traj = calc_trajectory(result.diff1);
	result.diff2 = calc_diff2(temp_sequence);
	result.diff2_traj = calc_trajectory(result.diff2);
	return result;
}

vector<vector<string> > Result::make_csv_data() const {
	vector<vector<string> > csv;
	csv.push_back(vector<string>({"name:"+name}));
	csv.push_back(vector<string>({"title:"+title}));
	csv.push_back(vector<string>({"xlabel:"+xlabel}));
	csv.push_back(vector<string>({"ylabel:"+ylabel}));
	csv.push_back(vector<string>({""}));
	csv.push_back(vector<string>({"similarity"}));
	for(double s:similarity){
		csv.push_back(vector<string>({to_string(s)}));
	}
	return csv;
}

vV calc_trajectory(vV &v){
	vV result(v.size() - 1);
	for(int i=0;i<v.size()-1;i++){
		double x = v[i+1].x - v[i].x;
		double y = v[i+1].y - v[i].y;
		double z = v[i+1].z - v[i].z;
		double t = v[i].t;
		result[i] = Vector(x, y, z, t);
	}
	return result;
}

vV calc_diff1(vV &v){
	vV result;
	for(int i=1;i<v.size()-1;i++){
		double h = 2 * (v[i+1].t - v[i-1].t);
		double x = (v[i+1].x - v[i-1].x) / h;
		double y = (v[i+1].y - v[i-1].y) / h;
		double z = (v[i+1].z - v[i-1].z) / h;
		result.push_back(Vector(x, y, z, v[i].t));
	}
	return result;
}

vV calc_diff2(vV &v){
	vV result;
	for(int i=1;i<v.size()-1;i++){
		double h = (v[i+1].t - v[i].t) * (v[i+1].t - v[i].t);
		double x = (v[i+1].x - 2 * v[i].x + v[i-1].x) / h;
		double y = (v[i+1].y - 2 * v[i].y + v[i-1].y) / h;
		double z = (v[i+1].z - 2 * v[i].z + v[i-1].z) / h;
		result.push_back(Vector(x, y, z, v[i].t));
	}
	return result;
}

vV moving_average(const int n, const vV& v){
	vV result;
	int length = v.size();
	vector<int> range;
	for(int i=-(n/2);i<=(n/2);i++){
		range.push_back(i);
	}
	for(int i=0;i<length;i++){
		double x = 0, y = 0, z = 0;
		int cnt = 0;
		for(int j : range){
			int index = i + j;
			if(index < 0 || index >= length){ continue; }
			x += v[index].x;
			y += v[index].y;
			z += v[index].z;
			cnt++;
		}
		result.push_back(Vector(x/cnt, y/cnt, z/cnt, v[i].t));
	}
	return result;
}

vector<vector<string> > csv_reader(const string filename){
	ifstream ifs(filename);
	if(ifs.fail()){
		cerr << "File does not exist.\n";
		exit(0);
	}
	vector<vector<string> > data;
	string line, cell;
	int line_no = 0;
	while(getline(ifs, line)){
		replace(line.begin(), line.end(), ',', ' ');
		stringstream buf(line);
		vector<string> tmp;
		while(buf >> cell)
			tmp.push_back(cell);
		data.push_back(tmp);
		line_no++;
	}
	return data; 
}

void csv_writer(const string filename, const vector<vector<string> >& data){
	ofstream ofs(filename);
	for(vector<string> line_data:data){
		int n = 1;
		for(string cell:line_data){
			if(n == line_data.size()){
				ofs << cell << endl;
			}else{
				ofs << cell << ',';
				n++;
			}
		}
	}
}  

void output_vector(const string filename, vV &data){
	vector<vector<string> > string_mat;
	vector<double> x, y, z;
	string_mat.push_back(vector<string>({"time","x","y","z"}));
	for(Vector vec:data){
		vector<string> temp = {to_string(vec.t), to_string(vec.x), to_string(vec.y), to_string(vec.z)};
		string_mat.push_back(temp);
	}
	csv_writer(filename, string_mat);
}

vector<Joint> csv_to_joint(const string filename, const int filter_n){
	ifstream ifs(filename);
	vector<Joint> data(JOINT_NUM);
	string line;
	double time, x, y, z;
	int line_no = 0;
	while(getline(ifs, line)){
		replace(line.begin(), line.end(), ',', ' ');
		stringstream buf(line);
		if(line_no == 0){ line_no++; continue; }
		buf >> time;
		for(int i=0;i<JOINT_NUM;i++){
			buf >> x >> y >> z;
			data[i].push_back(x, y, z, time);
		}
	}
	for(int i=0;i<JOINT_NUM;i++){
		data[i].name = NAME_MAP.at(i);
		data[i].sequence = moving_average(filter_n, data[i].sequence);
		data[i].trajectory = calc_trajectory(data[i].sequence);
		data[i].diff1 = calc_diff1(data[i].sequence);
		data[i].diff1_traj = calc_trajectory(data[i].diff1);
		data[i].diff2 = calc_diff2(data[i].sequence);
		data[i].diff2_traj = calc_trajectory(data[i].diff2);
	}
	return data;
}

vector<Joint> joint_to_bone(const vector<Joint>& joints){
	vector<Joint> bones(BONE_NUM);
	bones[0] = joints[1] - joints[0]; // Head to Neck
	bones[1] = joints[2] - joints[1]; // Neck to Torso
	bones[2] = joints[3] - joints[1]; // Neck to Left Shoulder
	bones[3] = joints[6] - joints[1]; // Neck to Right Shoulder
	bones[4] = joints[9] - joints[2]; // Torso to Left Hip
	bones[5] = joints[12] - joints[2]; // Torso to Right Hip
	bones[6] = joints[10] - joints[9]; // Left Hip to Left Knee
	bones[7] = joints[11] - joints[10]; // Left Knee to Left Foot
	bones[8] = joints[13] - joints[12]; // Right Hip to Right Knee
	bones[9] = joints[14] - joints[13]; // Right Knee to Right Foot
	for(int i=0;i<BONE_NUM;i++){
		bones[i].name = BONE_MAP.at(i);
	}
	return bones;
}

