#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf/tf.h>
#include <auv_msgs/NavSts.h>
#include <sensor_msgs/Image.h>
#include<fstream>
#include <boost/program_options.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv/highgui.h>
#include <opencv/cv.h>


using namespace std;
namespace po =boost::program_options;


tf::Transformer g_transformer;

string g_matfile_prefix="scan";
string g_matfile_suffix = ".txt";

/***TODO: add static transform!!!!!!!!!!!!!!!!!!!!!!!!
 *
 *
 *
 *
 *
 *
 *
 *
 *
 * ********/

void saveImage(const sensor_msgs::ImageConstPtr &msg, const string &prefix, const int &number){
	string image_file_name = prefix + std::to_string(number) + ".jpg";
	cv_bridge::CvImageConstPtr cv_img = cv_bridge::toCvShare(msg, "bgr8");
	if(cv_img){
		if(cv::imwrite(image_file_name, cv_img->image)){
			cout<<"writing image "<<image_file_name.c_str()<<endl;
		}else{
			cerr<<"writing to image "<<image_file_name.c_str()<<" failed"<<endl;
		}
	}
}

void saveTransformAndImages(const sensor_msgs::ImageConstPtr &left_image, const sensor_msgs::ImageConstPtr &right_image, int &scan_number){
	string *error_string;
	tf::StampedTransform transform;


	if(g_transformer.canTransform(right_image->header.frame_id,"local", right_image->header.stamp)){


		//g_transformer.lookupTransform(right_image->header.frame_id, "local", right_image->header.stamp, transform);
		g_transformer.lookupTransform("local", right_image->header.frame_id,  right_image->header.stamp, transform);
	}else{
		cout<<"could not get required time"<<endl;
		ros::Time latest_time;
		g_transformer.getLatestCommonTime(right_image->header.frame_id, "local", latest_time, error_string);
		if(latest_time.isZero()){
			cerr<<"could not transform at all, did not get any nav message till now"<<endl;
			return;
		}
		//g_transformer.lookupTransform(right_image->header.frame_id, "local", latest_time, transform);
		g_transformer.lookupTransform("local", right_image->header.frame_id, latest_time, transform);

	}


	tf::Vector3 origin = transform.getOrigin();
	tf::Quaternion rot_quat = transform.getRotation();
	tf::Matrix3x3 mat(rot_quat);


	/**
	 * write to file scan{scan_number}.txt
	 * */


	scan_number++;
	string mat_filename(g_matfile_prefix);
	mat_filename += std::to_string(scan_number);
	mat_filename += g_matfile_suffix;


	ofstream mat_file(mat_filename);
	/**
	  not using loop since have to add translation

	  */
	tf::Vector3 row1 =mat.getRow(0);
	tf::Vector3 row2 =mat.getRow(1);
	tf::Vector3 row3 =mat.getRow(2);
	mat_file<<row1.x()<<" "<<row1.y()<<" "<<row1.z()<<" "<<origin.x()<<endl;
	mat_file<<row2.x()<<" "<<row2.y()<<" "<<row2.z()<<" "<<origin.y()<<endl;
	mat_file<<row3.x()<<" "<<row3.y()<<" "<<row3.z()<<" "<<origin.z()<<endl;
	mat_file<<"0 0 0 1"<<endl;

	mat_file.close();
	cout<<"finished creating file"<<mat_filename<<endl;
	if(!left_image || !right_image){
		cerr<<"Should not be happing"<<endl;
	}else{
	//	saveImage(left_image, "left", scan_number);
		//saveImage(right_image, "right", scan_number);
	}

}

int main(int argc, char **argv){
	/**
	  read multiple bag files
	  when we get navsts message convert into tf, add the static transform to it. add to transformer
	  when we get the image message, get the transform at the time
	  */

	po::options_description desc("Allowed options");
	desc.add_options()("help,h", "produce help message")
			("nav_bag,n", po::value<string>(), "navigation_bag")
			("nav_topic,t", po::value<string>(), "nav_topic")
			("left_image_topic,l", po::value<string>(), "left_image_topic")
			("right_image_topic,r", po::value<string>(), "right_image_topic")
			("image_bags,b", po::value<vector<string>>(), "image_bags")
			("start_time,s", po::value<double>(), "start_time")
			("duration,u", po::value<double>(), "duration");
	po::positional_options_description pos_options;
	pos_options.add("image_bags", -1);


	po::variables_map vm;
	po::store(po::command_line_parser(argc, argv).options(desc).positional(pos_options).run(), vm);
	po::notify(vm);


	if(vm.count("help")){
		cout<<desc<<endl;
		return 1;
	}

	string tf_bag_filename; // = argv[1];
	string nav_msg_topic; // = argv[2];
	string left_image_topic, right_image_topic; // = argv[3];

	vector<string> image_bag_files;
	vector<std::shared_ptr<rosbag::Bag>> images_bags;

	double init_time_seconds(-1.0);
	double duration_seconds(-1.0);

	if(vm.count("start_time")){
		init_time_seconds = vm["start_time"].as<double>();
		cout<<"start_time" <<init_time_seconds<<endl;
	}else{

	}

	if(vm.count("duration")){
		duration_seconds = vm["duration"].as<double>();
		cout<<"duration "<<duration_seconds<<endl;
	}

	if(vm.count("nav_bag")){
		tf_bag_filename =vm["nav_bag"].as<string>();
	}else{
		cerr<<"should set nav_bag"<<endl;
		return -1;
	}

	if(vm.count("nav_topic")){
		nav_msg_topic = vm["nav_topic"].as<string>();
	}else{
		cerr<<"need to have nav_topic"<<endl;
		return -1;
	}


	if(vm.count("left_image_topic")){
		left_image_topic = vm["left_image_topic"].as<string>();
	}else{
		cerr<<"need image topic"<<endl;
		return -1;
	}
	if(vm.count("right_image_topic")){
		right_image_topic = vm["right_image_topic"].as<string>();
	}else{
		cerr<<"need image topic"<<endl;
		return -1;
	}
	if(vm.count("image_bags")){
		image_bag_files = vm["image_bags"].as<vector<string>>();


	}else{
		cerr<<"should specify image_bags"<<endl;
		return -1;
	}



	//make the static transform directly for now
	tf::Quaternion bqt;
	bqt.setRPY(0, 0, -M_PI/2.0);
	tf::Transform bumblebee_transform( bqt, tf::Vector3(-0.57, -0.12, 0.23));



	rosbag::Bag tf_bag(tf_bag_filename, rosbag::bagmode::Read);

	int scan_number = 0;

	images_bags.resize(image_bag_files.size());
	for(size_t i =0; i < image_bag_files.size(); ++i){
		cout<<image_bag_files[i]<<endl;
		images_bags[i].reset(new rosbag::Bag(image_bag_files[i], rosbag::bagmode::Read));
	}

	cout<<"finished opening bag"<<endl;
	std::vector<string> topics;
	topics.push_back(nav_msg_topic);
	topics.push_back(left_image_topic);
	topics.push_back(right_image_topic);

	rosbag::View view; //(tf_bag, rosbag::TopicQuery(nav_msg_topic));

	view.addQuery(tf_bag); //just to get the times

	//ros::Time initial_time = init_time_seconds>=0 ? view.getBeginTime() + ros::Duration(init_time_seconds) : view.getBeginTime();
	ros::Time initial_time = view.getBeginTime();
	if(init_time_seconds>0){
		//cout<<"before setting initial_time is "<<initial_time.toNSec()<<endl;
		//cout<<"init time seconds is "<<ros::Duration(init_time_seconds).toSec()<<endl;
		initial_time +=ros::Duration(init_time_seconds);
		//cout<<"initial time is set to "<<initial_time.toNSec()<<endl;
	}

	ros::Time end_time = view.getEndTime();
	if(duration_seconds >=0){
		end_time = initial_time + ros::Duration(duration_seconds);
	}



	cout<<"view begin_time" <<view.getBeginTime().toNSec()<<" view end time "<<view.getEndTime().toNSec()<<endl;
	view.addQuery(tf_bag, rosbag::TopicQuery(nav_msg_topic), initial_time, end_time);


	cout<<"initial time "<<initial_time.toNSec()<<" end_time"<<end_time.toNSec()<<endl;

//	view.addQuery(tf_bag, rosbag::TopicQuery(nav_msg_topic));
	//view.addQuery(*images_bags[0],rosbag::TopicQuery(topics), initial_time, end_time);
	for(const auto bag:images_bags){
		view.addQuery(*bag, rosbag::TopicQuery(topics), initial_time, end_time);
	}
	cout<<"finished making querries. total size"<<view.size()<<endl;


	//transformer.setExtrapolationLimit(ros::Duration(0.2));
	tf::StampedTransform transform;

	bool synchronized=true;

	sensor_msgs::ImageConstPtr left_image;
	sensor_msgs::ImageConstPtr right_image;

	for(const auto it:view){

		if(it.getTopic()== nav_msg_topic){
			/**
			  if message is nav_msg, get tf

			*/

			auv_msgs::NavSts::ConstPtr msg = it.instantiate<auv_msgs::NavSts>();
			transform.setOrigin(tf::Vector3(msg->position.north, msg->position.east, msg->position.depth));
			//transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
			transform.setRotation(tf::createQuaternionFromRPY(msg->orientation.roll, msg->orientation.pitch, msg->orientation.yaw));
			transform.frame_id_ = "local";
			//transform.child_frame_id_ = "base_link_filt";
			transform.child_frame_id_ = "bumblebee";
			transform.stamp_ = msg->header.stamp;
			transform*=bumblebee_transform;
			g_transformer.setTransform(transform);

		}else if (it.getTopic() ==right_image_topic){
			/**
			 * @brief we synchronize left and right
			 * if previous image is synchronized
			 *	store this image and set synchronized to false
			 * if not synchronized
			 *	check if timestamp matches
			 *		if matches
			 *			find current transform, set synchronized to true, save both images
			 *		else
			 *			store this image, hopefully gets synchronized by the other image
			 */
			right_image = it.instantiate<sensor_msgs::Image>();

			if (synchronized){

				synchronized =false;
			}else{
				if(left_image && left_image->header.stamp == right_image->header.stamp){
					saveTransformAndImages(left_image, right_image, scan_number);
					synchronized = true;
				}else{
					cerr<<"we've probably skpped a message, hopefully this message is synchronized"<<endl;
				}
			}

		}else if (it.getTopic() == left_image_topic){

			/**
			 * @brief check latest common time between the images and position,
			 *
			 */
			left_image = it.instantiate<sensor_msgs::Image>();

			if (synchronized){

				synchronized =false;
			}else{
				if(right_image && left_image->header.stamp == right_image->header.stamp){
					saveTransformAndImages(left_image, right_image, scan_number);
					synchronized = true;
				}else{
					cerr<<"we've probably skpped a message, hopefully this message is synchronized"<<endl;
				}
			}
		}
	}
	return 0;
}
