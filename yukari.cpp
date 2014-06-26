#include "research.hpp"

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
		
		REP(i, JOINT_NUM){
			string joint_name = master[i].name;
			double bm_sim = DPmatching(master[i].trajectory, before[i].trajectory);
			double am_sim = DPmatching(master[i].trajectory,  after[i].trajectory);
			cout << joint_name << " : " << bm_sim << " -> " << am_sim << " diff: " << bm_sim - am_sim << endl;
		}
		
		/*
		REP(i, JOINT_NUM){
			output_vector("output/master_" + master[i].name + "_seq.csv", master[i].sequence);
			output_vector("output/master_" + master[i].name + "_diff1.csv", master[i].diff1);
			output_vector("output/master_" + master[i].name + "_diff2.csv", master[i].diff2);
		}
		
		REP(i, JOINT_NUM){
			output_vector("output/after_" + after[i].name + "_seq.csv", after[i].sequence);
			output_vector("output/after_" + after[i].name + "_diff1.csv", after[i].diff1);
			output_vector("output/after_" + after[i].name + "_diff2.csv", after[i].diff2);
		}
		REP(i, JOINT_NUM){
			output_vector("output/before_" + before[i].name + "_seq.csv", before[i].sequence);
			output_vector("output/before_" + before[i].name + "_diff1.csv", before[i].diff1);
			output_vector("output/before_" + before[i].name + "_diff2.csv", before[i].diff2);
		}
		cout << endl << endl;
		REP(i, BONE_NUM){
			string bone_name = master_bones[i].name;
			double bm_sim = DPmatching(master_bones[i].trajectory, before_bones[i].trajectory);
			double am_sim = DPmatching(master_bones[i].trajectory,  after_bones[i].trajectory);
			cout << bone_name << " : " << bm_sim << " -> " << am_sim << " diff: " << bm_sim - am_sim << endl;
		}
		REP(i, BONE_NUM){
			output_vector("output/master_" + master_bones[i].name + "_seq.csv", master_bones[i].sequence);
			output_vector("output/master_" + master_bones[i].name + "_diff1.csv", master_bones[i].diff1);
			output_vector("output/master_" + master_bones[i].name + "_diff2.csv", master_bones[i].diff2);
		}
		*/
	}
	return 0;
}
