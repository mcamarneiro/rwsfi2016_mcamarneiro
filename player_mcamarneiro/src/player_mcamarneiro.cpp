#include <ros/ros.h>
#include <rwsfi2016_libs/player.h>

using namespace std;
using namespace ros;
int n=0;

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
                move(msg.max_displacement, angToPrey);
            }
        }
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
