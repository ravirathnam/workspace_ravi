#include <iostream>
#include <tf/transform_datatypes.h>
#include <fstream>
#include <pcl/point_cloud.h>//for pointcloud
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <cmath>

using namespace std;

inline void Matrix4ToEuler(const double *alignxf,
                                  double *rPosTheta,
                                  double *rPos = 0)
{
  
  double _trX, _trY;

  // Calculate Y-axis angle 
  if(alignxf[0] > 0.0) {
    rPosTheta[1] = asin(alignxf[8]);
  } else {
    rPosTheta[1] = M_PI - asin(alignxf[8]);
  }

  double  C    =  cos( rPosTheta[1] );
  if ( fabs( C ) > 0.005 )  {                 // Gimball lock? 
    _trX      =  alignxf[10] / C;             // No, so get X-axis angle 
    _trY      =  -alignxf[9] / C;
    rPosTheta[0]  = atan2( _trY, _trX );
    _trX      =  alignxf[0] / C;              // Get Z-axis angle 
    _trY      = -alignxf[4] / C;
    rPosTheta[2]  = atan2( _trY, _trX );
  } else {                                    // Gimball lock has occurred 
    rPosTheta[0] = 0.0;                       // Set X-axis angle to zero 
    _trX      =  alignxf[5];  //1                // And calculate Z-axis angle 
    _trY      =  alignxf[1];  //2
    rPosTheta[2]  = atan2( _trY, _trX );
  }
  
  rPosTheta[0] = rPosTheta[0];
  rPosTheta[1] = rPosTheta[1];
  rPosTheta[2] = rPosTheta[2];

  if (rPos != 0) {
    rPos[0] = alignxf[12];
    rPos[1] = alignxf[13];
    rPos[2] = alignxf[14];
  }
}

inline double radianToDegree(double rad){
	return rad*180.0/M_PI;
}
void convertAndWriteScanToFile(const char *file_name, const pcl::PointCloud<pcl::PointXYZRGB> &cloud){
	ofstream out_scan1(file_name);
	for(const auto it:cloud){
		if(!isfinite(it.x)){
			continue; //it is enough to check just x
		}
		
		int rgb = it.rgb;
		uint8_t r = (rgb >> 16) & 0x0000ff;
		uint8_t g = (rgb >> 8)  & 0x0000ff;
		uint8_t b = (rgb)     & 0x0000ff;
//		out_scan1<<it.x*1000<<" "<<it.y*1000<<" "<<-it.z*1000<<" "<<(int)it.r<<" "<<(int)it.g<<" "<<(int)it.b<<endl;
		out_scan1<<it.x*1000<<" "<<it.y*1000<<" "<<-it.z*1000<<" "<<(int)it.r<<" "<<(int)it.g<<" "<<(int)it.b<<endl;
	}
	out_scan1.close();
	
}
int main(int argc, char **argv){
	
	if(argc !=6){
		cerr<<"USAGE:"<<argv[0]<<" scan_file in_mat_file out_scan_file out_pose_file out_frames_file"<<endl;
		return -1;
	}
	const char* in_scan1_file = argv[1];
//	const char* in_scan2_file = argv[2];
	const char* in_mat_file = argv[2];
	const char* out_scan1_file = argv[3];
//	const char* out_scan2_file = argv[5];
	const char* out_pose_file = argv[4];
	const char* out_frames_file = argv[5];

	ifstream mat_file(in_mat_file);

	ofstream pose_file(out_pose_file);


	if(!mat_file.good()){
		cerr<<"Could not open file "<<argv[2]<<endl;
		return -1;
	}


	double tf_mat[16];

	// file will be in row major homogenous matrix
	for(int i=0; i < 16; ++i){

		mat_file>>tf_mat[i];
	}

	double col_tf_mat[16];//column major matrix
	double trans[3];
   double rot[3];

	for(int r=0; r < 4; ++r){
		for(int c=0; c<4; ++c){
			col_tf_mat[c*4+r] = tf_mat[r*4+c];
		}
	}

	//ensuring that after reflecting the z axis, the transformation should still work. therefore Rot(z) remains the same, but Rot(x) and Rot(y) are negated	

	col_tf_mat[0] = col_tf_mat[0];
   col_tf_mat[1] = col_tf_mat[1];	
   col_tf_mat[2] = -col_tf_mat[2];	
   col_tf_mat[3] = col_tf_mat[3];	
   col_tf_mat[4] = col_tf_mat[4];	
   col_tf_mat[5] = col_tf_mat[5];	
   col_tf_mat[6] = -col_tf_mat[6];	
   col_tf_mat[7] = col_tf_mat[7];	
   col_tf_mat[8] = -col_tf_mat[8];	
   col_tf_mat[9] = -col_tf_mat[9];	
   col_tf_mat[10] = col_tf_mat[10];	
   col_tf_mat[11] = col_tf_mat[11];	
	col_tf_mat[12] = col_tf_mat[12]*1000;
	col_tf_mat[13] = col_tf_mat[13]*1000;
	col_tf_mat[14] = -col_tf_mat[14]*1000;

	Matrix4ToEuler(col_tf_mat, rot, trans);



	//get translation
/*	double tx,ty,tz;
	tx = tf_mat[3];
	ty= tf_mat[7];
	tz = tf_mat[11];


	//ignoring translation only getting rotation
	tf::Matrix3x3 m(tf_mat[0], tf_mat[1], tf_mat[2], tf_mat[4], tf_mat[5], tf_mat[6], tf_mat[8], tf_mat[9], tf_mat[10]);
	double rx,ry, rz;
//	m.getRPY(rx, ry, rz);
	m.getEulerYPR(rz, ry, rx);
	//we will flip the z axis, so, rx, and ry will also have to be flipped

	tz = -tz;
	rx = -rx;
	ry= -ry;

	pose_file<<tx<<" "<<ty<<" "<<tz<<endl;

	pose_file<<radianToDegree(rx)<<" "<<radianToDegree(ry)<<" "<<radianToDegree(rz)<<endl;	
*/

	pose_file<<trans[0]<<" "<<trans[1]<<" "<<trans[2]<<endl;

	pose_file<<radianToDegree(rot[0])<<" "<<radianToDegree(rot[1])<<" "<<radianToDegree(rot[2])<<endl;	

	ofstream frames_file(out_frames_file);

	for(int i=0; i < 16; ++i){
		frames_file<<col_tf_mat[i]<<" ";
	}
	frames_file<<2<<endl;
	//now read the pcd files

	pcl::PCDReader pcl_reader;
	pcl::PointCloud<pcl::PointXYZRGB> scan1;
//	pcl::PointCloud<pcl::PointXYZRGB> scan2;
	
	pcl_reader.read(in_scan1_file, scan1);
//	pcl_reader.read(in_scan2_file, scan2);

	convertAndWriteScanToFile(out_scan1_file, scan1);
//	convertAndWriteScanToFile(out_scan2_file, scan2);
	mat_file.close();
	pose_file.close();
	return 0;
}
