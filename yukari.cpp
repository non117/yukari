#include "research.hpp"

void perpendicular(vector<Result>& results, const vector<Joint>& master, const vector<Joint>& before, const vector<Joint>& after){
	vector<int> numbers = {0,1,2,3,6,9,10,11,12,13,14};
	vector<vector<int> > comb = combination(numbers, 3);
	vector<Result> result;
	for(auto sample : comb){
		int i = sample[0];
		int j = sample[1];
		int k = sample[2];
		Joint perpend_master = ((master[i] - master[j]) && (master[i] - master[k])).normalized();
		Joint perpend_before = ((before[i] - before[j]) && (before[i] - before[k])).normalized();
		Joint perpend_after = ((after[i] - after[j]) && (after[i] - after[k])).normalized();
		string name = "perpendicular of "+NAME_MAP.at(i)+" "+NAME_MAP.at(j)+" "+NAME_MAP.at(k);
		results.push_back(Result(name, perpend_master, perpend_before, perpend_after));
	}
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
		vector<Result> results, locals;
		foot_coord(results, master, before, after);
		torso_coord(results, master, before, after);
		perpendicular(locals, master, before, after);
		sort(results.begin(), results.end());
		sort(locals.begin(), locals.end());
		int i = 0;
		for(Result r: locals){
			if(i == 20)
				break;
			//cout << i << ", " << r.name << ", " << r.before << ", " << r.after << ", " << r.before - r.after << endl;
			i++;
		}
		i = 0;
		for(Result r: results){
			if(r.after - r.before > 0 || i==100)
				break;
			cout << i << ", " << r.name << ", " << r.before << ", " << r.after << ", " << r.before - r.after << endl;
			i++;
		}
	}
	return 0;
}
