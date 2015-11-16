#include <iostream>
#include <fstream>
#include <Eigen/Dense>

using namespace std;
int main(int argc, char **argv){

	if(argc !=6){
		cerr<<"USAGE: "<<argv[0]<<" resfiles_prefix start_num end_num increment posefile_prefix"<<endl;
		return -1;
	}

	string res_file_prefix = argv[1];
	int start_num = atoi(argv[2]);
	int end_num = atoi(argv[3]);
	int increment = atoi(argv[4]);
	string posefile_prefix = argv[5];

	char resfile_name[30];
	Eigen::Matrix4f Ti_g=Eigen::Matrix4f::Identity(4,4);
	Eigen::Matrix4f Ti2_i1(4,4);

	for(int scan1=start_num; scan1 <= end_num-increment; scan1+=increment){
		int scan2 = scan1  + increment;
		sprintf(resfile_name, "%s%03d%03d.txt", res_file_prefix.c_str(), scan1, scan2);
		ifstream tf_file(resfile_name);
		if(!tf_file.good()){
			cerr<<"could not find file "<<resfile_name<<endl;
			continue;
		}
		float tf_data[16];
		for(int i=0; i < 16; ++i){
			tf_file>>tf_data[i];
		}
		for(int r=0; r<4; ++r){
			for(int c=0; c<4; ++c){
				Ti2_i1(r,c) = tf_data[r*4+c]; 
			}
		}
		tf_file.close();
		Ti2_i1.data();

		Ti_g *= Ti2_i1;
		ofstream gtf_file(posefile_prefix+std::to_string(scan2) + ".txt");
		float*tfg_data = Ti_g.data();
		for(int r=0; r<4; ++r){
			for(int c=0; c<4; ++c){
				gtf_file<<Ti_g(r,c)<<" ";
			}
			gtf_file<<endl;
		}
		gtf_file.close();	
	}

	return 0;
}
