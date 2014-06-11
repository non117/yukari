#include "research.hpp"

Vector::Vector(const double x, const double y, const double z, const double t) :
	x(x), y(y), z(z), t(t){}

const double Vector::norm() const{
	double sum = x * x + y * y + z * z;
	return sqrt(sum);
}

double Vector::operator * (const Vector& right) const{
	double up = x * right.x + y * right.y + z * right.z;
	double down = norm() * right.norm();
	double res = up / down;
	if(res < 0)
		res = 0;
	return 1 - res;
}

void Joint::push_back(const double x, const double y, const double z, const double t){
	sequence.push_back(Vector(x, y, z, t));
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

double DPmatching2(const vector<Vector>& v1_orig, const vector<Vector>& v2_orig){
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
	return path_matrix[m-1][n-1].cost;
}

double DPmatching(const vector<Vector>& v1_orig, const vector<Vector>& v2_orig){
	auto v1 = convert_to_trajectory(v1_orig);
	auto v2 = convert_to_trajectory(v2_orig);
	double error = 100.0f;
	int m = v1.size(), n = v2.size();
	vector<vector<double> > cos_(m, vector<double>(n));
	REP(i, m){
		REP(j, n){
			double sim = v1[i] * v2[j];
			//if(sim < 0){ sim = 0; }
			cos_[i][j] = sim;
		}
	}
	vector<vector<double> > similarity(m, vector<double>(n));
	int loop = min(n, m);
	// S(0,0) = sim(0,0), S(i,0) = S(0,i) = 100.0f
	REP(i, m)
		similarity[i][0] = error;
	REP(j, n)
		similarity[0][j] = error;
	similarity[0][0] = cos_[0][0];
	vector<vector<Node> > path(m, vector<Node>(n)); 
	path[0][0] = Node(cos_[0][0], Point(-1, -1), Point(0, 0));
	for(int i=1;i<loop;i++){
		REP(j, m-i){
			double n1 = similarity[i+j-1][i-1] + cos_[i+j][i];
			double n2 = similarity[i+j-1][i] + cos_[i+j][i];
			double n3 = similarity[i+j][i-1] + cos_[i+j][i];
			if( n1 <= n2 && n1 <= n3){
				similarity[i+j][i] = n1;
				// trans_num[num2 + num, num] = trans_num[num2 + num - 1, num - 1] + 1;
				path[i+j][i] = Node(n1, Point(i+j-1, i-1), Point(i+j, i));
			}else if (n2 <= n1 && n2 <= n3){
				similarity[i+j][i] = n2;
				// trans_num[num2 + num, num] = trans_num[num2 + num - 1, num] + 1;  
				 path[i+j][i] = Node(n2, Point(i+j-1, i), Point(i+j, i)); 
			}else if (n3 <= n1 && n3 <= n2){
				similarity[i+j][i] = n3;
				// trans_num[num2 + num, num] = trans_num[num2 + num, num - 1] + 1; 
				 path[i+j][i] = Node(n3, Point(i+j, i-1), Point(i+j, i)); 
			} 
		}
		REP(j, n-i){
			double n1 = similarity[i-1][i+j-1] + cos_[i][i+j];
			double n2 = similarity[i-1][i+j] + cos_[i][i+j];
			double n3 = similarity[i][i+j-1] + cos_[i][i+j];
			if( n1 <= n2 && n1 <= n3){
				similarity[i][i+j] = n1;
				// trans_num[num, num2 + num] = trans_num[num - 1, num2 + num - 1] + 1; 
				path[i][i+j] = Node(n1, Point(i-1, i+j-1), Point(i, i+j)); 
			}else if( n2 <= n1 && n2 <= n3){
				similarity[i][i+j] = n2;
				// trans_num[num, num2 + num] = trans_num[num - 1, num2 + num] + 1; 
				path[i][i+j] = Node(n2, Point(i-1, i+j), Point(i, i+j));
			}else if( n3 <= n1 && n3 <= n2){
				similarity[i][i+j] = n3;
				// trans_num[num, num2 + num] = trans_num[num, num2 + num - 1] + 1; 
				path[i][i+j] = Node(n3, Point(i, i+j-1), Point(i, i+j));
			} 
		}
	}
	vector<double> costs;
	int x = m - 1;
	int y = n - 1;
	double prev_cost = path[x][y].cost; 
	while (x != -1) {
		Node p = path[x][y];
		costs.push_back(prev_cost - p.cost);
		prev_cost = p.cost;
		x = p.prev.x;
		y = p.prev.y;
	}
	reverse(costs.begin(), costs.end());
	
	 
	for(auto a : costs)
		cout << a << ",";
	cout << endl;
	
	
	return similarity[m-1][n-1]/costs.size();
}


