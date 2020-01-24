/*
 Copyright (c) 2020, Robot Control and Pattern Recognition Group, Warsaw University of Technology
 All rights reserved.
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
     * Neither the name of the Warsaw University of Technology nor the
       names of its contributors may be used to endorse or promote products
       derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "identification_action_msgs/IdentificationAction.h"
#include "identification_action_msgs/IdentificationGoal.h"

#include <rtt/RTT.hpp>
#include <rtt/Component.hpp>
#include <rtt/Logger.hpp>

#include "rtt_actionlib/rtt_actionlib.h"
#include "rtt_actionlib/rtt_action_server.h"

using namespace RTT;

class IdentificationActionServer : public RTT::TaskContext
{
private:
	typedef actionlib::ServerGoalHandle<identification_action_msgs::IdentificationAction> GoalHandle;
	typedef boost::shared_ptr<const identification_action_msgs::IdentificationGoal> Goal;

	rtt_actionlib::RTTActionServer<identification_action_msgs::IdentificationAction> as_;
	GoalHandle active_goal_;

	RTT::OutputPort<unsigned int> port_identification_command_;

    bool goalActive_;

public:
	explicit IdentificationActionServer(const std::string& name)
		: TaskContext(name)
		, goalActive_(false)
	{
		this->ports()->addPort("identification_command_OUTPORT", port_identification_command_);

		as_.addPorts(this->provides());
		as_.registerGoalCallback(boost::bind(&IdentificationActionServer::goalCB, this, _1));
		as_.registerCancelCallback(boost::bind(&IdentificationActionServer::cancelCB, this, _1));
	}

	~IdentificationActionServer(){}

	bool startHook() 
	{
		if (as_.ready())
		{
			as_.start();
		} 
		else 
		{
			return false;
		}
		return true;
	}

    void updateHook() 
    {
	    identification_action_msgs::IdentificationResult res;	
		if (goalActive_) 
		{
			res.error_code = identification_action_msgs::IdentificationResult::SUCCESSFUL;
			active_goal_.setSucceeded(res);
			goalActive_ = false;
		}	
	}

private:
	void goalCB(GoalHandle gh)
	{	
	    identification_action_msgs::IdentificationResult res;	

		Goal g = gh.getGoal();        

        if (g->action == identification_action_msgs::IdentificationGoal::ACTION_FIRST_MEASUREMENT_BEFORE_OBJECT_IS_GRASPED) 
        {
            port_identification_command_.write(1);
        }
        else if (g->action == identification_action_msgs::IdentificationGoal::ACTION_SECOND_MEASUREMENT_BEFORE_OBJECT_IS_GRASPED)  
        {
        	port_identification_command_.write(2);
        }
        else if (g->action == identification_action_msgs::IdentificationGoal::ACTION_FIRST_MEASUREMENT_AFTER_OBJECT_IS_GRASPED)  
        {
        	port_identification_command_.write(3);
        }
        else if (g->action == identification_action_msgs::IdentificationGoal::ACTION_SECOND_MEASUREMENT_AFTER_OBJECT_IS_GRASPED)  
        {
        	port_identification_command_.write(4);
        }
        else
        { 
			res.error_code = identification_action_msgs::IdentificationResult::ERROR;
		    gh.setRejected(res);
		    return;
        }
        
		gh.setAccepted();
		active_goal_ = gh;

		goalActive_ = true;
        return;
	}

	void cancelCB(GoalHandle gh)
	{
		if (active_goal_ == gh) 
		{
			active_goal_.setCanceled();
		}
	}
};

ORO_CREATE_COMPONENT(IdentificationActionServer)
