#include "ros/ros.h"
#include "std_msgs/Float32.h"

using namespace std;
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>


/*
	kbhit()
	$BI8=`F~NO(B(stdin)$B$r%j%"%k%?%$%`(B(RT)$B=hM}$r$9$k$?$a$N%a%C%;!<%83NG'4X?t(B
	getchar$B$d(Bcin$B$G$O(BEnter$B6h@Z$j$G$d$C$HF~NO$5$l$k$N$G!"%-!<$,2!$5$l$F$$$k$+$I$&$+H=CG$G$-$J$$!#(B
	$B$3$N4X?t$G$O!"(Bstdin$B$r>/$72C9)$7$F$3$l$i$r$G$-$k$h$&$K$7$F!"%-!<$,$"$l$P#1!"$J$1$l$P#0=PNO$9$k!#(B
http://i2blog.matrix.jp/index.php?UID=1479357418
*/
int kbhit(void)
{
	struct termios oldt, newt;
	int ch;
	int oldf;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
	
	ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);
	
	if (ch != EOF) {
		ungetc(ch, stdin);
		return 1;
	}

	return 0;
}


class Robot
{
	ros::NodeHandle n;
	ros::Publisher ms0;
	ros::Publisher ms1;
	ros::Publisher ms2;
	ros::Publisher ms3;

public:
	Robot(void);
	void begin(void);

private:
};

Robot::Robot(void)
{
	ms0 = n.advertise<std_msgs::Float32>("/ms1/velocity",10);
	ms1 = n.advertise<std_msgs::Float32>("/ms2/velocity",10);
	ms2 = n.advertise<std_msgs::Float32>("/ms3/velocity",10);
	ms3 = n.advertise<std_msgs::Float32>("/ms4/velocity",10);
}

void Robot::begin(void)
{
	float speed_param=10.0;	//$B%-!<J}8~$X$NB.EY(B
	float x_speed;
	float y_speed;

	ros::Rate loop_rate(1);

	while(ros::ok()){
		char key;
		if(kbhit() == 0)key = 0;
		else key = getchar();
		fflush(STDIN_FILENO);

		switch(key){			//$B%-!<H=JL!!0lHL%2!<%`;EMM(B
		case 'a':
			x_speed = -1*speed_param;
			y_speed = 0;
			break;
		case 'w':
			x_speed = 0;
			y_speed = speed_param;
			break;
		case 's':
			x_speed = 0;
			y_speed = -1*speed_param;
			break;
		case 'd':
			x_speed = speed_param;
			y_speed = 0;
			break;
		default:
			x_speed = 0;
			y_speed = 0;
			break;
		}
		ROS_INFO_STREAM("test "<<(int)key);
		ros::spinOnce();//$B%3!<%k%P%C%/4X?t8F$S=P$7(B
		loop_rate.sleep();
	}
}

int main(int argc,char **argv)
{
	ros::init(argc,argv,"mekanamu_4");
	Robot r;
	r.begin();
}
