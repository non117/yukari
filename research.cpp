#include "research.hpp"

double Vector::operator * (const Vector& right) const{
	double up = x * right.x + y * right.y + z * right.z;
	double down = norm() * right.norm();
	double res = up / down;
	if(res < 0)
		res = 0;
	return 1 - res;
}

vector<Joint> csv_to_joint(const string filename){
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
		REP(i, JOINT_NUM){
			buf >> x >> y >> z;
			data[i].push_back(x, y, z, time);
		}
	}
	REP(i, JOINT_NUM)
		data[i].name = NAME_MAP.at(i);
	return data;
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
			if(n == line_data.size())
				ofs << cell << endl;
			else
				ofs << cell << ',';
		}
	}
}

vector<Vector> convert_to_trajectory(const vector<Vector>& v){
	vector<Vector> result(v.size()-1);
	REP(i, v.size()-1){
		double x = v[i+1].x - v[i].x;
		double y = v[i+1].y - v[i].y;
		double z = v[i+1].z - v[i].z;
		double t = v[i].t;
		result[i] = Vector(x, y, z, t);
	}
	return result;
}

double DPmatching(const vector<Vector>& v1_orig, const vector<Vector>& v2_orig){
	auto v1 = convert_to_trajectory(v1_orig);
	auto v2 = convert_to_trajectory(v2_orig);
	int m = v1.size(), n = v2.size(), x, y;
	vector<vector<Node> > path_matrix(m, vector<Node>(n));
	Node point(v1[0]*v2[0], Point(-1,-1), Point(0,0));
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
			float add_cost = v1[nx] * v2[ny];
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




