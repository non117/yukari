#include "research.hpp"

vector<Result> cross(const unordered_map<int, string> maps, const vector<Joint>& master, const vector<Joint>& before, const vector<Joint>& after){
	vector<int> numbers;
	int n = maps.size();
	for(int i=0;i<n;i++)
		numbers.push_back(i);
	vector<vector<int> > comb = combination(numbers, 2);
	vector<Result> result;
	for(auto sample : comb){
		int first = sample[0];
		int second = sample[1];
		Joint cross_master = master[first] && master[second];
		Joint cross_before = before[first] && before[second];
		Joint cross_after = after[first] && after[second];
		auto res_mb = DPmatching(cross_master.trajectory, cross_before.trajectory);
		auto res_ma = DPmatching(cross_master.trajectory, cross_after.trajectory);
		auto v_mb = DPmatching(cross_master.diff1_traj, cross_before.diff1_traj);
		auto v_ma = DPmatching(cross_master.diff1_traj, cross_after.diff1_traj);
		//cout << v_mb.second[20];
		Result temp = Result(cross_master.name, first, second, res_mb.first, res_ma.first, res_mb.second, res_ma.second);
		Result tempv = Result(cross_master.name + " velocity", first, second, v_mb.first, v_ma.first, v_mb.second, v_ma.second);
		result.push_back(temp);
		//result.push_back(tempv);
	}
	return result;
}

int main(int argc, char* argv[]){
	int filter_n = 5;
	if(argc == 4){
		string master_file = argv[1];
		string before_file = argv[2];
		string after_file  = argv[3];
		vector<Joint> master = csv_to_joint(master_file, filter_n);
		vector<Joint> before = csv_to_joint(before_file, filter_n);
		vector<Joint> after  = csv_to_joint( after_file, filter_n);
		vector<Joint> master_bones = joint_to_bone(master);
		vector<Joint> before_bones = joint_to_bone(before);
		vector<Joint> after_bones = joint_to_bone(after);
		vector<Result> results = cross(BONE_MAP, master_bones, before_bones, after_bones);
		sort(results.begin(), results.end());
		master_bones[results[9].first];
		for(int i=0;i<45;i++){
			auto r = results[i];
			cout << i << ", " << r.name << ", " << r.before << ", " << r.after << endl;
		}
		
		REP(i, JOINT_NUM){
			string joint_name = master[i].name;
			//double bm_sim = DPmatching(master[i].trajectory, before[i].trajectory);
			//double am_sim = DPmatching(master[i].trajectory,  after[i].trajectory);
		}
	}
	return 0;
}
