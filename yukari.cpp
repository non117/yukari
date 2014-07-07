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
		Result temp = Result(cross_master.name, res_mb, res_ma);
		Result tempv = Result(cross_master.name + " velocity", v_mb, v_ma);
		result.push_back(temp);
		//result.push_back(tempv);
	}
	return result;
}

void foot_coord(vector<Result>& results, const vector<Joint>& master, const vector<Joint>& before, const vector<Joint> after){
	for(int i=0;i<JOINT_NUM;i++){
		if( find(EXCLUDES.begin(), EXCLUDES.end(), i) == EXCLUDES.end()){
			if(i == 11)
				continue;
			Joint lhoot_master = master[i] - master[11];
			Joint lhoot_before = before[i] - before[11];
			Joint lhoot_after = after[i] - after[11];
			auto lhoot_mb = DPmatching(lhoot_master.trajectory, lhoot_before.trajectory);
			auto lhoot_ma = DPmatching(lhoot_master.trajectory, lhoot_after.trajectory);
			auto lhoot_mb_v = DPmatching(lhoot_master.diff1_traj, lhoot_before.diff1_traj);
			auto lhoot_ma_v = DPmatching(lhoot_master.diff1_traj, lhoot_after.diff1_traj);
			results.push_back(Result(lhoot_master.name, lhoot_mb, lhoot_ma));
			results.push_back(Result(lhoot_master.name+" velocity", lhoot_mb_v, lhoot_ma_v));
			if(i == 14)
				continue;
			Joint rhoot_master = master[i] - master[14];
			Joint rhoot_before = before[i] - before[14];
			Joint rhoot_after = after[i] - after[14];
			auto rhoot_mb = DPmatching(rhoot_master.trajectory, rhoot_before.trajectory);
			auto rhoot_ma = DPmatching(rhoot_master.trajectory, rhoot_after.trajectory);
			auto rhoot_mb_v = DPmatching(rhoot_master.diff1_traj, rhoot_before.diff1_traj);
			auto rhoot_ma_v = DPmatching(rhoot_master.diff1_traj, rhoot_after.diff1_traj);
			results.push_back(Result(rhoot_master.name, rhoot_mb, rhoot_ma));
			results.push_back(Result(rhoot_master.name+" velocity", rhoot_mb_v, rhoot_ma_v));
		}
	}
}

void torso_coord(vector<Result>& results, const vector<Joint>& master, const vector<Joint>& before, const vector<Joint> after){
	for(int i=0;i<JOINT_NUM;i++){
		if( find(EXCLUDES.begin(), EXCLUDES.end(), i) == EXCLUDES.end()){ 
			if(i == 2)
				continue;
			auto mb = DPmatching(master[i].trajectory, before[i].trajectory);
			auto ma = DPmatching(master[i].trajectory, after[i].trajectory);
			auto mb_v = DPmatching(master[i].diff1_traj, before[i].diff1_traj);
			auto ma_v = DPmatching(master[i].diff1_traj, after[i].diff1_traj);
			results.push_back(Result(master[i].name, mb, ma));
			results.push_back(Result(master[i].name+" velocity", mb_v, ma_v));
		}
	}
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
		vector<Result> results;
		foot_coord(results, master, before, after);
		torso_coord(results, master, before, after);
		sort(results.begin(), results.end());
		for(int i=0;i<45;i++){
			auto r = results[i];
			if(r.after - r.before > 0)
				break;
			cout << i << ", " << r.name << ", " << r.before << ", " << r.after << endl;
		}
	}
	return 0;
}
