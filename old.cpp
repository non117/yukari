#include "research.hpp"

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
	return similarity[m-1][n-1]/costs.size();
}  
