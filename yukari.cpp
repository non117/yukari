#include "research.hpp"

// TODO : similarity( variable, data no ).csvを出力するやつ
// 軸周り座標系のやつ
// 時間で切るやつ
// 角度
// 金曜日までに

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

void output_all_similarity(vector<string>& data){
	string master = data[0];
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
		vector<string> data;
		for(int i=2;i<argc;i++)
			data.push_back(argv[i]);
		output_all_similarity(data);
	}else if(option == "-izukura" && argc == 5){
		string master_file = argv[2];
		string before_file = argv[3];
		string after_file  = argv[4];
		vector<Joint> master = csv_to_joint(master_file, filter_n);
		vector<Joint> before = csv_to_joint(before_file, filter_n);
		vector<Joint> after  = csv_to_joint( after_file, filter_n);
		//vector<Joint> master_bones = joint_to_bone(master);
		//vector<Joint> before_bones = joint_to_bone(before);
		//vector<Joint> after_bones = joint_to_bone(after);
		izukura_method(master, before, after);
	}
	return 0;
}
