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

/* arr  ---> Input Array
   n      ---> Size of input array
   r      ---> Size of a combination to be printed
   index  ---> Current index in data[]
   data ---> Temporary array to store current combination
   i      ---> index of current element in arr[]     */
void combinationloop(const vector<int>& arr, int r, int index, vector<int>& data, int i, vector<vector<int> >& result){
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

vector<vector<int> > combination(const vector<int>& arr, int r){
	vector<vector<int> > result;
	vector<int> data(r);
	combinationloop(arr, r, 0, data, 0, result);
	return result;
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

double DPmatching(const vV& v1, const vV& v2){
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




