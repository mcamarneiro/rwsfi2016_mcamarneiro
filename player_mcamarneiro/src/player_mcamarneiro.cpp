#include <ros/ros.h>
#include <rwsfi2016_libs/player.h>

using namespace std;
using namespace ros;


/**
 * @brief MyPlayer extends class Player, i.e., there are additional things I can do with MyPlayer and not with any Player, e.g., to order a movement.
 */


class MyPlayer: public rwsfi2016_libs::Player
{
  public: 

    /**
     * @brief Constructor, nothing to be done here
     * @param name player name
     * @param pet_name pet name
     */
    MyPlayer(string player_name, string pet_name="/dog"): Player(player_name, pet_name){};

    void play(const rwsfi2016_msgs::MakeAPlay& msg)
    {
      //Custom play behaviour. Now I will win the game

        //move(msg.max_displacement, M_PI/30);
        double hunt=1000, run=10;
        bool running=false;
        for(int i=0; i< 3; i++)
        {
            if(getDistanceToPlayer(hunters_team->players[i])<3.5)
            {
                running=true;
                double angToMove= -getAngleToPLayer(hunters_team->players[i]);
                if(angToMove>=M_PI/30 )
                        move(msg.max_displacement, M_PI/30);
                else if(angToMove<=-M_PI/30 )
                        move(msg.max_displacement, -M_PI/30);
            }
        }
        if (!running)
        {
            double minDist=100;
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
            if(angToPrey>M_PI/30)
               angToPrey=M_PI/30;
            else if(angToPrey<-M_PI/30)
                angToPrey=-M_PI/30;

            move(msg.max_displacement, angToPrey);
        }
      //Behaviour follow the closest prey
      //move(msg.max_displacement, M_PI/30);
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
