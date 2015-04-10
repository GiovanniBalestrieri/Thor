#include "arduino_com_node_imu.h"

int send_status_imu[] = {0, 0, 0};
int send_status_odom[] = {0, 0};

sensor_msgs::Imu imu_msg;
nav_msgs::Odometry odom;

ros::Publisher imu_pub;
ros::Publisher odom_pub;

int main(int argc, char **argv){
	ros::init(argc, argv, "imu_bridge");	
	ros::NodeHandle n;
	ros::Rate r(10);
		
	/**** COVARIANZE E COSTANTI ****/
	set_covariance();
	imu_msg.header.frame_id = "imu";
	odom.header.frame_id = "wheelodom";
	odom.child_frame_id = "base_footprint";
	
	/**** GESTIONE TOPIC ****/
	/* DEPRECATO */	
	/*
	ros::Subscriber quatsub = n.subscribe("quaternion", 1000, quat_callback);
	ros::Subscriber velsub = n.subscribe("ang_vel", 1000, vel_callback);
	ros::Subscriber accsub = n.subscribe("lin_accel", 1000, acc_callback);
	
	ros::Subscriber posesub = n.subscribe("odom_pose", 1000, pose_callback);
	ros::Subscriber twistsub = n.subscribe("odom_twist", 1000, twist_callback);
	*/
	imu_pub = n.advertise<sensor_msgs::Imu>("imu_data", 1000);
	odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1000);
	/*
	ros::spin();
	*/
	
	/**** COMUNICAZIONE SERIALE ****/
	char * serialport = "/dev/ttyACM0";
	cserial serial;
	if(serial.connect(serialport, 115200) < 0)
	{
		return 1;
	}
	int actualsequencenumber = 0;
	while(1)
	{
		/* protocollo di lettura: 
		 *	byte 0 		-> sequence_number
		 *	byte 1 		-> sensor_id
		 *	byte [2-5] 	-> msg_size
		 *	byte [6-fine]	-> data
		 */
		char * cmd = serial.serial_read(READ_ALL);
		if(cmd == NULL) 
		{
			actualsequencenumber = (actualsequencenumber+1)%256;
			r.sleep();
			continue;
		}
		/* parsing comando */
		char sequence_number = cmd[SEQNO_OFF];
		if(sequence_number < actualsequencenumber)
		{
			/* ricevuto un messaggio vecchio */
			free(cmd);
			r.sleep();
			continue;
		}
		else if(sequence_number > actualsequencenumber)
		{
			actualsequencenumber = sequence_number;
		}
		char sensor_id = cmd[SENSORID_OFF];
		int msg_size = cmd[MSGSIZE_OFF];
		char * data = cmd+DATA_OFF;
		if(sensor_id == IMU_SENSOR)
		{
			/* dati IMU */
			/* quaternion (4 float)
			 * ang_vel (3 float)
			 * lin_acc (3 float)
			 */
			 if(msg_size != 10*sizeof(float))
			 {
			 	/* dimensione sbagliata */
			 	/* ignoro il messaggio */
			 	free(cmd);
			 	actualsequencenumber = (actualsequencenumber+1)%256;
			 	r.sleep();
			 	continue;
			 }
			 float quaternion[QUAT_SIZE];
			 float ang_vel[VEC3_SIZE];
			 float lin_acc[VEC3_SIZE];
			 /* deserializzo i dati */
			 int offset = 0;
			 memcpy(quaternion, (data+offset), QUAT_SIZE*sizeof(float));
			 offset += QUAT_SIZE*sizeof(float);
			 memcpy(ang_vel, (data+offset), VEC3_SIZE*sizeof(float));
			 offset += VEC3_SIZE*sizeof(float);
			 memcpy(lin_acc, (data+offset), VEC3_SIZE*sizeof(float));
			 offset += VEC3_SIZE*sizeof(float);
			 
			 /* invio */
			 imu_quat(quaternion);
			 imu_vel(ang_vel);
			 imu_acc(lin_acc);
		}
		else if(sensor_id == ODOM_SENSOR)
		{
			/* dati Odometry */
			/* pose
			 *	position (3 float)
			 *	orientation (4 float)
			 * twist
			 *	linear (3 float)
			 *	angular (3 float)
			 */
			 if(msg_size != 10*sizeof(float))
			 {
			 	/* dimensione sbagliata */
			 	/* ignoro il messaggio */
			 	free(cmd);
			 	actualsequencenumber = (actualsequencenumber+1)%256;
			 	r.sleep();
			 	continue;
			 }
			 float position[VEC3_SIZE];
			 float orientation[QUAT_SIZE];
			 float twist_linear[VEC3_SIZE];
			 float twist_angular[VEC3_SIZE];
			 
			 int offset = 0;
			 memcpy(position, (data+offset), VEC3_SIZE*sizeof(float));
			 offset += VEC3_SIZE*sizeof(float);
			 memcpy(orientation, (data+offset), QUAT_SIZE*sizeof(float));
			 offset += QUAT_SIZE*sizeof(float);
			 memcpy(twist_linear, (data+offset), VEC3_SIZE*sizeof(float));
			 offset += VEC3_SIZE*sizeof(float);
			 memcpy(twist_angular, (data+offset), VEC3_SIZE*sizeof(float));
			 offset += VEC3_SIZE*sizeof(float);
			 
			 /*invio i dati*/
			 pose(position, orientation);
			 twist(twist_linear, twist_angular);
		}
		
		/* Aggiorno il numero di sequenza e addormento il processo */
		free(cmd);
		actualsequencenumber = (actualsequencenumber+1)%256;
		r.sleep();
	}
	serial.disconnect();
	return 0;
}

void set_covariance(){
	// Covarianze IMU
	imu_msg.orientation_covariance[0] = 1.0;
	imu_msg.orientation_covariance[1] = 0.0;
	imu_msg.orientation_covariance[2] = 0.0;
	imu_msg.orientation_covariance[3] = 0.0;
	imu_msg.orientation_covariance[4] = 1.0;
	imu_msg.orientation_covariance[5] = 0.0;
	imu_msg.orientation_covariance[6] = 0.0;
	imu_msg.orientation_covariance[7] = 0.0;	
	imu_msg.orientation_covariance[8] = 1.0;
	
	imu_msg.angular_velocity_covariance[0] = 1.0;
	imu_msg.angular_velocity_covariance[1] = 0.0;
	imu_msg.angular_velocity_covariance[2] = 0.0;
	imu_msg.angular_velocity_covariance[3] = 0.0;
	imu_msg.angular_velocity_covariance[4] = 1.0;
	imu_msg.angular_velocity_covariance[5] = 0.0;
	imu_msg.angular_velocity_covariance[6] = 0.0;
	imu_msg.angular_velocity_covariance[7] = 0.0;
	imu_msg.angular_velocity_covariance[8] = 1.0;

	imu_msg.linear_acceleration_covariance[0] = 1.0;
	imu_msg.linear_acceleration_covariance[1] = 0.0;
	imu_msg.linear_acceleration_covariance[2] = 0.0;
	imu_msg.linear_acceleration_covariance[3] = 0.0;
	imu_msg.linear_acceleration_covariance[4] = 1.0;
	imu_msg.linear_acceleration_covariance[5] = 0.0;
	imu_msg.linear_acceleration_covariance[6] = 0.0;
	imu_msg.linear_acceleration_covariance[7] = 0.0;
	imu_msg.linear_acceleration_covariance[8] = 1.0;
	
	//Covarianze odometry
	odom.pose.covariance[0] = 1.0;
	odom.pose.covariance[1] = 0.0;
	odom.pose.covariance[2] = 0.0;
	odom.pose.covariance[3] = 0.0;
	odom.pose.covariance[4] = 0.0;
	odom.pose.covariance[5] = 0.0;
	odom.pose.covariance[6] = 0.0;
	odom.pose.covariance[7] = 1.0;
	odom.pose.covariance[8] = 0.0;
	odom.pose.covariance[9] = 0.0;
	odom.pose.covariance[10] = 0.0;
	odom.pose.covariance[11] = 0.0;
	odom.pose.covariance[12] = 0.0;
	odom.pose.covariance[13] = 0.0;
	odom.pose.covariance[14] = 1.0;
	odom.pose.covariance[15] = 0.0;
	odom.pose.covariance[16] = 0.0;
	odom.pose.covariance[17] = 0.0;
	odom.pose.covariance[18] = 0.0;
	odom.pose.covariance[19] = 0.0;
	odom.pose.covariance[20] = 0.0;
	odom.pose.covariance[21] = 1.0;
	odom.pose.covariance[22] = 0.0;
	odom.pose.covariance[13] = 0.0;
	odom.pose.covariance[24] = 0.0;
	odom.pose.covariance[25] = 0.0;
	odom.pose.covariance[26] = 0.0;
	odom.pose.covariance[27] = 0.0;
	odom.pose.covariance[28] = 1.0;
	odom.pose.covariance[29] = 0.0;
	odom.pose.covariance[30] = 0.0;
	odom.pose.covariance[31] = 0.0;
	odom.pose.covariance[32] = 0.0;
	odom.pose.covariance[33] = 0.0;
	odom.pose.covariance[34] = 0.0;
	odom.pose.covariance[35] = 1.0;
	
	odom.twist.covariance[0] = 1.0;
	odom.twist.covariance[1] = 0.0;
	odom.twist.covariance[2] = 0.0;
	odom.twist.covariance[3] = 0.0;
	odom.twist.covariance[4] = 0.0;
	odom.twist.covariance[5] = 0.0;
	odom.twist.covariance[6] = 0.0;
	odom.twist.covariance[7] = 1.0;
	odom.twist.covariance[8] = 0.0;
	odom.twist.covariance[9] = 0.0;
	odom.twist.covariance[10] = 0.0;
	odom.twist.covariance[11] = 0.0;
	odom.twist.covariance[12] = 0.0;
	odom.twist.covariance[13] = 0.0;
	odom.twist.covariance[14] = 1.0;
	odom.twist.covariance[15] = 0.0;
	odom.twist.covariance[16] = 0.0;
	odom.twist.covariance[17] = 0.0;
	odom.twist.covariance[18] = 0.0;
	odom.twist.covariance[19] = 0.0;
	odom.twist.covariance[20] = 0.0;
	odom.twist.covariance[21] = 1.0;
	odom.twist.covariance[22] = 0.0;
	odom.twist.covariance[13] = 0.0;
	odom.twist.covariance[24] = 0.0;
	odom.twist.covariance[25] = 0.0;
	odom.twist.covariance[26] = 0.0;
	odom.twist.covariance[27] = 0.0;
	odom.twist.covariance[28] = 1.0;
	odom.twist.covariance[29] = 0.0;
	odom.twist.covariance[30] = 0.0;
	odom.twist.covariance[31] = 0.0;
	odom.twist.covariance[32] = 0.0;
	odom.twist.covariance[33] = 0.0;
	odom.twist.covariance[34] = 0.0;
	odom.twist.covariance[35] = 1.0;
	
}

void sendImu(int rec){
	send_status_imu[rec] = 1;
	if(send_status_imu[0] == 1 && send_status_imu[1] == 1 && send_status_imu[2] == 1){

		/**** INVIO MESSAGGIO IMU ****/
		imu_msg.header.stamp = ros::Time::now();
		imu_pub.publish(imu_msg);

		/**** RESET STATO DI RICEZIONE DATI ****/
		send_status_imu[0] = 0;
		send_status_imu[1] = 0;
		send_status_imu[2] = 0;
	}
}

void sendOdometry(int type){
	send_status_odom[type] = 1;
	if(send_status_odom[0] == 1 && send_status_odom[1] == 1){
		odom.header.stamp = ros::Time::now();
		odom_pub.publish(odom);
		send_status_odom[0] = 0;
		send_status_odom[1] = 0;

	}
}

/* FUNZIONI DI TRASFERIMENTO ARRAY - ROS_MSG */

void imu_quat(const float * q){
	if(q != NULL)
	{
		imu_msg.orientation.x = q[0];
		imu_msg.orientation.y = q[1];
		imu_msg.orientation.z = q[2];
		imu_msg.orientation.w = q[3];
	
		sendImu(QUATERNION_MSG);
	}
}

void imu_vel(const float * s){
	if(s != NULL)
	{
		imu_msg.angular_velocity.x = s[0];
		imu_msg.angular_velocity.y = s[1];
		imu_msg.angular_velocity.z = s[2];

		sendImu(VELOCITY_MSG);
	}
}

void imu_acc(const float * a){
	if(a != NULL)
	{
		imu_msg.linear_acceleration.x = a[0];
		imu_msg.linear_acceleration.y = a[1];
		imu_msg.linear_acceleration.z = a[2];
	
		sendImu(ACCELERATION_MSG);
	}
}

void pose(const float * p, const float * o){
	if(p != NULL && o != NULL)
	{
		odom.pose.pose.position.x = p[0];
		odom.pose.pose.position.y = p[1];
		odom.pose.pose.position.z = p[2];
	
		odom.pose.pose.orientation.x = o[0];
		odom.pose.pose.orientation.y = o[1];
		odom.pose.pose.orientation.z = o[2];
		odom.pose.pose.orientation.w = o[3];
	
	
		sendOdometry(POSE_MSG);
	}
}

void twist(const float * l, const float * a){
	if(l != NULL && a != NULL)
	{
		odom.twist.twist.linear.x = l[0];
		odom.twist.twist.linear.y = l[1];
		odom.twist.twist.linear.z = l[2];
	
		odom.twist.twist.angular.x = a[0];
		odom.twist.twist.angular.y = a[1];
		odom.twist.twist.angular.z = a[2];
	
		sendOdometry(TWIST_MSG);
	}
}

/* FUNZIONI DI COMUNICAZIONE CON ARDUINO */
/* DEPRECATE */
/*
void quat_callback(const geometry_msgs::Quaternion q){
	imu_msg.orientation.x = q.x;
	imu_msg.orientation.y = q.y;
	imu_msg.orientation.z = q.z;
	imu_msg.orientation.w = q.w;
	
	sendImu(QUATERNION_MSG);
}

void vel_callback(const geometry_msgs::Vector3 s){
	imu_msg.angular_velocity.x = s.x;
	imu_msg.angular_velocity.y = s.y;
	imu_msg.angular_velocity.z = s.z;

	sendImu(VELOCITY_MSG);
}

void acc_callback(const geometry_msgs::Vector3 a){
	imu_msg.linear_acceleration.x = a.x;
	imu_msg.linear_acceleration.y = a.y;
	imu_msg.linear_acceleration.z = a.z;
	
	sendImu(ACCELERATION_MSG);
}

void pose_callback(const geometry_msgs::Pose p){
	odom.pose.pose.position.x = p.position.x;
	odom.pose.pose.position.y = p.position.y;
	odom.pose.pose.position.z = p.position.z;
	
	odom.pose.pose.orientation.x = p.orientation.x;
	odom.pose.pose.orientation.y = p.orientation.y;
	odom.pose.pose.orientation.z = p.orientation.z;
	odom.pose.pose.orientation.w = p.orientation.w;
	
	
	sendOdometry(POSE_MSG);
}

void twist_callback(const geometry_msgs::Twist t){
	odom.twist.twist.linear.x = t.linear.x;
	odom.twist.twist.linear.y = t.linear.y;
	odom.twist.twist.linear.z = t.linear.z;
	
	odom.twist.twist.angular.x = t.angular.x;
	odom.twist.twist.angular.y = t.angular.x;
	odom.twist.twist.angular.z = t.angular.x;
	
	sendOdometry(TWIST_MSG);
}
*/
