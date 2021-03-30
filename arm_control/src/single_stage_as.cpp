#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <infrastructure_msgs/StageAction.h>
//insert any headers needed for arm movement

class SingleStageActionServer{
	protected:	
		//variables for nodehandle, action server, feedback and results
		ros::NodeHandle nh_;
		actionlib::SimpleActionServer<infrastructure_msgs::StageAction> as_;
		std::string action_name_;
		infrastructure_msgs::StageFeedback feedback_;
	    infrastructure_msgs::StageResult result_;

	public:
		//Starts action server with name given in main function and begins async spinner for moveit
		SingleStageActionServer(std::string name):
		as_(nh_, name, boost::bind(&SingleStageActionServer::executeCB, this, _1), false)
		, action_name_(name){
			as_.start();
			ros::AsyncSpinner spinner(1);
			spinner.start();	
		}

		void executeCB(const infrastructure_msgs::StageGoalConstPtr &goal){
			ROS_INFO("ACTION EXECUTING");
			
			//checks for preempt
		 	if (as_.isPreemptRequested() || !ros::ok()){
        		ROS_INFO("%s: Preempted", action_name_.c_str());
        		as_.setPreempted();
      		}

			if(goal->start == true){	
				//ENTER YOUR CODE HERE 



				ROS_INFO("SUCCESS");
				result_.result = true;
				as_.setSucceeded(result_);
			}
			//placeholder for testing purposes
			else{
				ROS_INFO("FAILED");
				result_.result = false;
				as_.setSucceeded(result_);
			}
		}

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "single_stage_as");

  SingleStageActionServer demo("single_stage_as");
  ros::spin();
  return 0;
}
