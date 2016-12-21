#include <ros/ros.h>
#include <rwsfi2016_libs/player.h>
#include <visualization_msgs/Marker.h>
#include <rwsfi2016_msgs/GameQuery.h>

using namespace std;
using namespace ros;
int n=0;

/**
 * @brief MyPlayer extends class Player, i.e., there are additional things I can do with MyPlayer and not with any Player, e.g., to order a movement.
 */


class MyPlayer: public rwsfi2016_libs::Player
{
  public: 
        Publisher publ;
        visualization_msgs::Marker marker;
        ros::ServiceServer service;
    /**
     * @brief Constructor, nothing to be done here
     * @param name player name
     * @param pet_name pet name
     */
    MyPlayer(string player_name, string pet_name="/dog"): Player(player_name, pet_name)
    {
        publ =node.advertise<visualization_msgs::Marker>( "/bocas", 0 );
        marker.header.frame_id = name;
        marker.header.stamp = ros::Time();
        marker.ns = name;
        marker.id = 0;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.z = 0.4;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        service = node.advertiseService(name + "/game_query", &MyPlayer::serviceCallback,this);
    };

    bool serviceCallback(rwsfi2016_msgs::GameQuery::Request &req, rwsfi2016_msgs::GameQuery::Response &res )
    {
        res.resposta="Hello";
        return true;
    }

    void play(const rwsfi2016_msgs::MakeAPlay& msg)
    {
        bool running=false, end=false;

        double xPos=0, yPos=0;
        xPos=getPose().getOrigin().x();
        yPos=getPose().getOrigin().y();
        double distCenter=sqrt(xPos*xPos + yPos*yPos);

        if(distCenter>=6.8)
        {
            n++;
            if(n<20)
            move(msg.max_displacement, M_PI/30);
            else
            move(msg.max_displacement, M_PI/90);
        }
        else
        {
            n=0;
            int minDist=10, closer=0;
            //Compute Minimum Distance to Hunters
            for(int i=0; i< 3; i++)
            {
                int tmp=getDistanceToPlayer(hunters_team->players[i]);
                if(tmp<minDist)
                {
                    minDist=tmp;
                    closer=i;
                }
            }
            //Start Running from hunters if any is closer than 2.5m
            if(minDist<2.0)
            {
                std::cout<<"Escaping"<<std::endl;
                marker.text="\nAte Logo!!!";
                double angToMove= -getAngleToPLayer(hunters_team->players[closer]);
                move(msg.max_displacement, angToMove);
                move(msg.max_displacement, 0);
            }
            //No Hunters nearby, Let's hunt the closer prey
            else
            {
                double minDist=20;
                int toPrey=0;
                for(int i=0; i<3; i++)
                {
                    double dist = getDistanceToPlayer(preys_team->players[i]);
                    if(dist < minDist)
                    {
                        minDist=dist;
                        toPrey=i;
                    }
                }
                double angToPrey=getAngleToPLayer(preys_team->players[toPrey]);
                std::cout<<"Hunting"<<std::endl;
                marker.text="\nAnda ca ao pai";
                move(msg.max_displacement, angToPrey);
            }
        }
        publ.publish(marker);
    }
};


/**
 * @brief The main function. All you need to do here is enter your name and your pets name
 * @param argc number of command line arguments
 * @param argv values of command line arguments
 * @return result
 */
int main(int argc, char** argv)
{
  // ------------------------
  //Replace this with your name
  // ------------------------
  string my_name = "mcamarneiro";
  string my_pet = "/turtle";

  //initialize ROS stuff
  ros::init(argc, argv, my_name);
  //Creating an instance of class MyPlayer
  MyPlayer my_player(my_name, my_pet);

  //Infinite spinning (until ctrl-c)
  ros::spin();
}
