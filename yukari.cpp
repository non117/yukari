#include "research.hpp"

int main(int argc, char* argv[]){
	if(argc == 4){
		string master_file = argv[1];
		string before_file = argv[2];
		string after_file  = argv[3];
		vector<Joint> master = csv_to_joint(master_file);
		vector<Joint> before = csv_to_joint(before_file);
		vector<Joint> after  = csv_to_joint( after_file);
		REP(i, JOINT_NUM){
			string joint_name = master[i].name;
			double bm_sim = DPmatching(master[i].sequence, before[i].sequence);
			double am_sim = DPmatching(master[i].sequence,  after[i].sequence);
			cout << joint_name << " : " << bm_sim << " -> " << am_sim << " diff: " << bm_sim - am_sim << endl;
		}
		/*
		cout << endl;
		REP(i, JOINT_NUM){
			string joint_name = master[i].name;
			double bm_sim = DPmatching2(master[i].sequence, before[i].sequence);
			double am_sim = DPmatching2(master[i].sequence,  after[i].sequence);
			cout << joint_name << " : " << bm_sim << " -> " << am_sim << " diff: " << bm_sim - am_sim << endl; 
		}
		*/
	}
	return 0;
}
