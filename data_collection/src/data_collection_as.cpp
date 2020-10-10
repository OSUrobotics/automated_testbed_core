#include <opencv2/videoio.hpp>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <infrastructure_msgs/DataCollectionAction.h>
#include <infrastructure_msgs/DoorSensors.h>
#include <infrastructure_msgs/DataTimestamps.h>
#include <string>
#include <opencv2/opencv.hpp>

//insert any headers needed for arm movement


class DataCollectionActionServer{
	protected:	
		//variables for nodehandle, action server, feedback and results
		ros::NodeHandle nh_;
		actionlib::SimpleActionServer<infrastructure_msgs::DataCollectionAction> as_;
		std::string action_name_;
		infrastructure_msgs::DataCollectionFeedback feedback_;
	    infrastructure_msgs::DataCollectionResult result_;
		std::string file_location;
		ros::Publisher data_stamp;
		int trial_count;

	public:
		//Starts action server with name given in main function and begins async spinner for moveit
		DataCollectionActionServer(std::string name):
		as_(nh_, name, boost::bind(&DataCollectionActionServer::executeCB, this, _1), false)
		, action_name_(name){
			trial_count = 1;
			//This is janky as hell dont do this, temporary until I can find a better solution
			ros::NodeHandle nh2("~");
			nh2.getParam("video_path", file_location);
			data_stamp = nh_.advertise<infrastructure_msgs::DataTimestamps>("data_timestamps", 1000);
			as_.start();
		}


		void executeCB(const infrastructure_msgs::DataCollectionGoalConstPtr &goal){

			infrastructure_msgs::DataTimestamps stamp;
			stamp.trial_number = trial_count;
			stamp.collection_start_time = ros::Time::now();


			if(goal->collection_start == true){
					ROS_INFO("DATA COLLECTION STARTED");

					cv::VideoCapture capture(0);

					if(capture.isOpened() == false){
						ROS_ERROR("CANNOT OPEN VIDEO CAMERA");	
						ROS_WARN("PROCEEDING WITHOUT RECORDING VIDEO");
						
						while(!as_.isPreemptRequested() && ros::ok()){};
					}

					int width = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_WIDTH));
					int height = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_HEIGHT));
					cv::Size frame(width, height);
					int fps = 30;
					int codec = CV_FOURCC('X','V','I','D');
					std::string file_name = "trial_" + std::to_string(trial_count) + ".avi";
					std::string	file_location2 = file_location + file_name;

					std::cout << file_location2 << std::endl;
					cv::VideoWriter video(file_location2, codec, fps, frame, true);
					
					if(video.isOpened() == false){
						ROS_ERROR("CANNOT SAVE VIDEO FILE");
					}

					while(!as_.isPreemptRequested() && ros::ok() && video.isOpened() == true){
						cv::Mat frame;	
						capture >> frame;
						video.write(frame);
						cv::imshow("FRAMES", frame);
					
					}

			}

			//checks for preempt
		 	if (as_.isPreemptRequested() || !ros::ok()){
				stamp.collection_end_time = ros::Time::now();
				stamp.total_time = (stamp.collection_end_time.toSec() - stamp.collection_start_time.toSec());
				data_stamp.publish(stamp);

				trial_count++;
        		ROS_INFO("%s: Preempted", action_name_.c_str());
        		as_.setPreempted();
      		}

			if(goal->collection_start == false){		
				stamp.collection_end_time = ros::Time::now();
				stamp.total_time = (stamp.collection_end_time.toSec() - stamp.collection_start_time.toSec());
				data_stamp.publish(stamp);

				trial_count = 0;
				ROS_INFO("DATA COLLECTION STOPPED");
				result_.collection_stop = true;
				as_.setSucceeded(result_);
			}
		}

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "data_collection_as");
   
  DataCollectionActionServer demo("data_collection_as");
  ros::spin();
  return 0;
}
