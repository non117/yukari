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
			double bm_sim = DPmatching(master[i].trajectory, before[i].trajectory);
			double am_sim = DPmatching(master[i].trajectory,  after[i].trajectory);
			cout << joint_name << " : " << bm_sim << " -> " << am_sim << " diff: " << bm_sim - am_sim << endl;
		}
		REP(i, JOINT_NUM){
			output_vector("output/master_" + master[i].name + "_seq__.csv", master[i].sequence);
			output_vector("output/master_" + master[i].name + "_diff1__.csv", master[i].diff1);
			output_vector("output/master_" + master[i].name + "_diff2__.csv", master[i].diff2);
		}
	}
	return 0;
}
