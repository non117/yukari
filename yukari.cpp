#include "research.hpp"

// TODO : 
// 軸周り座標系のやつ
// 時間で切るやつ
// 角度

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

void x_coord(vector<Result>& results, const vector<Joint>& master, const vector<Joint>& before, const vector<Joint> after, const int joint_no){
	for(int i=0;i<JOINT_NUM;i++){
		if( find(EXCLUDES.begin(), EXCLUDES.end(), i) == EXCLUDES.end()){
			if( i == joint_no)
				continue;
			Joint m = master[i] - master[joint_no];
			Joint b = before[i] - before[joint_no];
			Joint a = after[i] - after[joint_no];
			string name = NAME_MAP.at(joint_no) + " coordinate of " + NAME_MAP.at(i);
			results.push_back(Result(name, m, b, a));
			results.push_back(Result(name + " velocity", m, b, a, true));
		}
	}
}

pair<vector<string>, vector<double> > perpendicular_sim(const vector<Joint>& master, const vector<Joint>& student){
	vector<int> numbers = {0,1,2,3,6,9,10,11,12,13,14};
	vector<vector<int> > comb = combination(numbers, 3);
	vector<double> sims;
	vector<string> names;
	for(auto sample : comb){
		int i = sample[0];
		int j = sample[1];
		int k = sample[2];
		Joint perpend_master = ((master[i] - master[j])&&(master[i] - master[k])).normalized();
		Joint perpend_student = ((student[i] - student[j])&&(student[i] - student[k])).normalized();
		string name = "perpendicular of "+NAME_MAP.at(i)+" "+NAME_MAP.at(j)+" "+NAME_MAP.at(k);
		auto s = DPmatching(perpend_master.trajectory, perpend_student.trajectory);
		sims.push_back(s.first);
		names.push_back(name);
	}
	return make_pair(names, sims);
} 

pair<vector<string>, vector<double> > x_coord_sim(const vector<Joint>& master, const vector<Joint>& student, const int joint_no){
	vector<double> sims;
	vector<string> names;
	for(int i=0;i<JOINT_NUM;i++){
		if( find(EXCLUDES.begin(), EXCLUDES.end(), i) == EXCLUDES.end()){
			if( i == joint_no)
				continue;
			Joint m = master[i] - master[joint_no];
			Joint s = student[i] - student[joint_no];
			string name = NAME_MAP.at(joint_no) + " coordinate of " + NAME_MAP.at(i);
			auto sim = DPmatching(m.trajectory, s.trajectory);
			auto sim_v = DPmatching(m.diff1_traj, s.diff1_traj);
			sims.push_back(sim.first);
			sims.push_back(sim_v.first);
			names.push_back(name);
			names.push_back(name + " velocity");
		}
	} 
	return make_pair(names, sims);
}

void izukura_method(vector<Joint>& master, vector<Joint>& before, vector<Joint>& after){
	vector<Result> results, locals;
	x_coord(results, master, before, after, 11); // LeftFoot
	x_coord(results, master, before, after, 14); // RightFoot
	x_coord(results, master, before, after, 2); // Torso
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

// 時刻分割は、それぞれのファイルをどこで時分割するか人間が決定し、メモしておく

void output_all_similarity(vector<string>& filenames){
	vector<Joint> master = csv_to_joint(filenames[0], 5);
	vector<string> header = {"filenames"};
	vector<vector<string> > results;
	for(int i=1;i<filenames.size();i++){
		vector<Joint> student = csv_to_joint(filenames[i], 5);
		auto lfootcoord = x_coord_sim(master, student, 11);
		auto rfootcoord = x_coord_sim(master, student, 14);
		auto torsocoord = x_coord_sim(master, student, 2);
		auto perpend = perpendicular_sim(master, student);
		if(i == 1){
			concat(header, lfootcoord.first);
			concat(header, rfootcoord.first);
			concat(header, torsocoord.first);
			concat(header, perpend.first);
			results.push_back(header);
		}
		vector<string> temp;
		temp.push_back(filenames[i]);
		concat(temp, map_to_string(lfootcoord.second));
		concat(temp, map_to_string(rfootcoord.second));
		concat(temp, map_to_string(torsocoord.second));
		concat(temp, map_to_string(perpend.second));
		results.push_back(temp);
	}
	csv_writer("all_similarity.csv", results);
}

void output_raw_csv(vector<string>& filenames){
	string delim ("/");
	for(string& filename : filenames){
		vector<string> list_string;
		boost::split(list_string, filename, boost::is_any_of(delim));
		vector<Joint> joints = csv_to_joint(filename, 5);
		for(Joint& joint : joints){
			joint.write_csv("raw/" + list_string[1] + "_" + joint.name);
		}
	}
}

int main(int argc, char* argv[]){
	int filter_n = 5;
	// -all master.csv, hoge.csv
	string option;
	if(argc > 1){
		option = argv[1];
	}else{
		return 0;
	}
	if(option == "-all"){
		vector<string> filenames;
		for(int i=2;i<argc;i++)
			filenames.push_back(argv[i]);
		output_all_similarity(filenames);
	}else if(option == "-izukura" && argc == 5){
		string master_file = argv[2];
		string before_file = argv[3];
		string after_file  = argv[4];
		vector<Joint> master = csv_to_joint(master_file, filter_n);
		vector<Joint> before = csv_to_joint(before_file, filter_n);
		vector<Joint> after  = csv_to_joint( after_file, filter_n);
		izukura_method(master, before, after);
	}else if(option == "-raw"){
		vector<string> filenames;
		for(int i=2;i<argc;i++)
			filenames.push_back(argv[i]);
		output_raw_csv(filenames);
	}
	return 0;
}
