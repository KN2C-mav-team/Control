#include "main.h"
#include "Control.h"
int decode_flag_mavlink_usart=0;
ch2int con;
int16_t mpc_data[2];
int16_t mpc_data_2[2];
uint8_t quality;
uint8_t buff_usart[34];

mavlink_system_t                   mavlink_system;
mavlink_optical_flow_t             OPTICAL_FLOW;
mavlink_battery_status_t           BATT_STATUS;
mavlink_manual_setpoint_t          RC_SETPOINT;
mavlink_sim_state_t                SIM_STATE ;
mavlink_attitude_t                 ATTITUDE ;
mavlink_altitude_t                 ALTITUDE ;
mavlink_rc_channels_t              RC_RAW_PPM_CHANNELS; // The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%
mavlink_rc_channels_raw_t          RC_CHANNELS_RAW ;
mavlink_rc_channels_scaled_t       RC_CHANNEL_SCALED ;
mavlink_mission_item_t             MISSION_ITEM ; //Message encoding a mission item. This message is emitted to announce the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See also http://qgroundcontrol.org/mavlink/waypoint_protocol.
mavlink_mission_request_t          MISSION_REQUEST ;//Request the information of the mission item with the sequence number seq. The response of the system to this message should be a MISSION_ITEM message. http://qgroundcontrol.org/mavlink/waypoint_protocol
mavlink_mission_set_current_t      MISSION_SET_CURRENT ;//Set the mission item with sequence number seq as current item. This means that the MAV will continue to this mission item on the shortest path (not following the mission items in-between).
mavlink_mission_current_t          MISSION_CURRENT; //Message that announces the sequence number of the current active mission item. The MAV will fly towards this mission item.
mavlink_mission_request_list_t     MISSION_REQUEST_LIST; //Request the overall list of mission items from the system/component.
mavlink_mission_count_t            MISSION_COUNT ; //This message is emitted as response to MISSION_REQUEST_LIST by the MAV and to initiate a write transaction. The GCS can then request the individual mission item based on the knowledge of the total number of MISSIONs.
mavlink_local_position_ned_cov_t   LOCAL_POSITION_NED_COV ; //The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
mavlink_mission_item_int_t         MISSION_ITEM_INT  ;//
mavlink_scaled_imu_t               SCALED_IMU ; //The RAW IMU readings for the usual 9DOF sensor setup. This message should contain the scaled values to the described units
mavlink_attitude_quaternion_t      ATTITUDE_QUATERNION;
mavlink_highres_imu_t              HIGHRES_IMU ; //The IMU readings in SI units in NED body frame
mavlink_control_system_state_t     CONTROL_SYSTEM_STATE ;//The smoothed, monotonic system state used to feed the control loops of the system


 mavlink_message_t msg;
 mavlink_status_t status;
 
 
 void encode_mavlink_packet(void)
{
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	uint16_t len;
	mavlink_system.sysid = 20;                   ///< ID 20 for this airplane
	mavlink_system.compid = MAV_COMP_ID_ALL;     ///< The component sending the message is the IMU, it could be also a Linux process

	
  //shrue meghdar dehi moteghayer haye mavlink*********************************************	
//  RC_SETPOINT.pitch      = RC.Pitch*(PI/180);
//	RC_SETPOINT.roll       = RC.Roll*(PI/180);
//  RC_SETPOINT.thrust     = RC.Throttle;
//  RC_SETPOINT.yaw        = RC.Yaw*(PI/180);	
//	RC_SETPOINT.mode_switch= Run_State;
//	RC_SETPOINT.manual_override_switch =
//	RC_SETPOINT.time_boot_ms =
	//***************************************************************************************
//	CONTROL_SYSTEM_STATE.x_acc =  //X acceleration in body frame
//	CONTROL_SYSTEM_STATE.y_acc = 
//	CONTROL_SYSTEM_STATE.z_acc =
//	CONTROL_SYSTEM_STATE.x_pos =  //X position in local frame
//	CONTROL_SYSTEM_STATE.y_pos =
//	CONTROL_SYSTEM_STATE.z_pos = 
//	CONTROL_SYSTEM_STATE.pitch_rate = //Angular rate in pitch axis
//	CONTROL_SYSTEM_STATE.roll_rate  = //Angular rate in roll axis
//	CONTROL_SYSTEM_STATE.yaw_rate   = //Angular rate in yaw axis
//	CONTROL_SYSTEM_STATE.x_vel      = //X velocity in body frame
//	CONTROL_SYSTEM_STATE.y_vel      =
//	CONTROL_SYSTEM_STATE.z_vel      =
//	CONTROL_SYSTEM_STATE.vel_variance = //Variance of body velocity estimate
//	CONTROL_SYSTEM_STATE.airspeed = -1 //	Airspeed, set to -1 if unknown
//	CONTROL_SYSTEM_STATE.pos_variance = //Variance in local position
//	CONTROL_SYSTEM_STATE.q = //The attitude, represented as Quaternion
//	CONTROL_SYSTEM_STATE.time_usec = //Timestamp (micros since boot or Unix epoch)
	//***************************************************************************************
  HIGHRES_IMU.xacc=  Mpu.acc_x;
	HIGHRES_IMU.yacc=  Mpu.acc_y;
	HIGHRES_IMU.zacc=  Mpu.acc_z;
	HIGHRES_IMU.xgyro= Mpu.gyro_x;
	HIGHRES_IMU.ygyro= Mpu.gyro_y;
	HIGHRES_IMU.zgyro= Mpu.gyro_z;
	//***************************************************************************************

	ATTITUDE_QUATERNION.q1=Mahony.q0;
	ATTITUDE_QUATERNION.q2=Mahony.q1;
	ATTITUDE_QUATERNION.q3=Mahony.q2;
	ATTITUDE_QUATERNION.q4=Mahony.q3;
	//***************************************************************************************
//	SCALED_IMU.xacc=Mpu.acc_x;
//	SCALED_IMU.yacc=Mpu.acc_y;
//	SCALED_IMU.ymag=Mpu.acc_z;
//	
//	SCALED_IMU.xgyro=Mpu.gyro_x;
//	SCALED_IMU.ygyro=Mpu.gyro_y;
//	SCALED_IMU.zgyro=Mpu.gyro_z;
	
	//****************************************************************************************
//	LOCAL_POSITION_NED_COV.ax= //X Acceleration (m/s^2)
//	LOCAL_POSITION_NED_COV.ay=
//	LOCAL_POSITION_NED_COV.az=
//	LOCAL_POSITION_NED_COV.covariance=//Covariance matrix upper right triangular (first nine entries are the first ROW, next eight entries are the second row, etc.)
//	LOCAL_POSITION_NED_COV.estimator_type= //	Timestamp (microseconds since UNIX epoch) in UTC. 0 for unknown. Commonly filled by the precision time source of a GPS receiver.
//	LOCAL_POSITION_NED_COV.time_boot_ms= //Timestamp (milliseconds since system boot). 0 for system without monotonic timestamp
//	LOCAL_POSITION_NED_COV.time_utc= //	Timestamp (microseconds since UNIX epoch) in UTC. 0 for unknown. Commonly filled by the precision time source of a GPS receiver.
//	LOCAL_POSITION_NED_COV.vx= //X Speed (m/s)
//	LOCAL_POSITION_NED_COV.vy=
//	LOCAL_POSITION_NED_COV.vz=
//	LOCAL_POSITION_NED_COV.x= //	X Position
//	LOCAL_POSITION_NED_COV.y=
//	LOCAL_POSITION_NED_COV.z= 
	//****************************************************************************************
//	MISSION_COUNT.count =1;//Number of mission items in the sequence
//	MISSION_COUNT.target_component =0;
//	MISSION_COUNT.target_system=20;
	//****************************************************************************************
//	MISSION_REQUEST_LIST.target_component=0;
//	MISSION_REQUEST_LIST.target_system=20;
	//****************************************************************************************
//	MISSION_CURRENT.seq=1;
	//**********************************************************************************
//	MISSION_SET_CURRENT.seq= 1;
//	MISSION_SET_CURRENT.target_component=0;
//	MISSION_SET_CURRENT.target_system=20;
	//**************************************************************************************
//	MISSION_REQUEST.seq = 1 ;
//	MISSION_REQUEST.target_component =0 ; //Component which should execute the command, 0 for all components
//	MISSION_REQUEST.target_system = 20 ;
	//*****************************************************************************************
//	Message encoding a mission item. 
//	This message is emitted to announce the presence of a mission item and to set a mission item on the system.
//	The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude.
//	Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU).
//	See also http://qgroundcontrol.org/mavlink/waypoint_protocol
//	MISSION_ITEM.autocontinue= //	autocontinue to next wp
	MISSION_ITEM.command = MAV_CMD_NAV_TAKEOFF_LOCAL; //The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs 
	MISSION_ITEM.current = 1 ;//false:0, true:1
	MISSION_ITEM.frame =MAV_FRAME_LOCAL_NED;//The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
//	MISSION_ITEM.param1 = MAV_CMD_NAV_TAKEOFF;//PARAM1, see MAV_CMD enum
//	MISSION_ITEM.param2 =MAV_CMD_NAV_TAKEOFF;//PARAM2, see MAV_CMD enum
//	MISSION_ITEM.param3 =MAV_CMD_NAV_TAKEOFF;//PARAM3, see MAV_CMD enum
//	MISSION_ITEM.param4 =MAV_CMD_NAV_TAKEOFF;//PARAM4, see MAV_CMD enum
//Mission Param #1	Minimum pitch (if airspeed sensor present), desired pitch without sensor [rad]
//Mission Param #2	Empty
//Mission Param #3	Takeoff ascend rate [ms^-1]
//Mission Param #4	Yaw angle [rad] (if magnetometer or another yaw estimation source present), ignored without one of these
//Mission Param #5	Y-axis position [m]
//Mission Param #6	X-axis position [m]
//Mission Param #7	Z-axis position [m]
//	MISSION_ITEM.seq = //Sequence
	//MISSION_ITEM.target_component = //Component ID
	MISSION_ITEM.target_system = 20 ;//System ID
	MISSION_ITEM.x = 77;//PARAM5 / local: x position, global: latitude
	MISSION_ITEM.y =77;//PARAM6 / y position: global: longitude
	MISSION_ITEM.z =77; //PARAM7 / z position: global: altitude (relative or absolute, depending on frame.
  //*****************************************************************************************	
//	SIM_STATE.roll  = Roll.point*(PI/180);
//	SIM_STATE.pitch = Pitch.point*(PI/180);
//	SIM_STATE.yaw   = Yaw.point*(PI/180);
//	SIM_STATE.xacc  = Mpu.acc_x;
//	SIM_STATE.yacc  = Mpu.acc_y;
//  SIM_STATE.zacc  = Mpu.acc_z;
//	SIM_STATE.xgyro = Mpu.gyro_x;
//	SIM_STATE.zgyro = Mpu.gyro_z;
//	SIM_STATE.ygyro = Mpu.gyro_y;
//  SIM_STATE.q1    = Mahony.q0;
//	SIM_STATE.q2    = Mahony.q1;
//	SIM_STATE.q3    = Mahony.q2;
//	SIM_STATE.q4    = Mahony.q3;
//	SIM_STATE.alt   = 
//	SIM_STATE.lat   =
//	SIM_STATE.lon   =
//	SIM_STATE.std_dev_horz =
//	SIM_STATE.std_dev_vert= =
//	SIM_STATE.vd    =
//	SIM_STATE.ve    =
//	SIM_STATE.vn    =
	//********************************************************************************************
//	ATTITUDE.pitch  = Pitch.point*(PI/180);
//	ATTITUDE.roll   = Roll.point*(PI/180);
//	ATTITUDE.yaw    = Yaw.point*(PI/180);
//	ATTITUDE.pitchspeed   =
//	ATTITUDE.rollspeed    =
//	ATTITUDE.yawspeed     = 
//	ATTITUDE.time_boot_ms =
  //********************************************************************************************
  OPTICAL_FLOW.flow_x       =100;// optical_par.delta_X; //Flow in pixels * 10 in x-sensor direction (dezi-pixels)
	OPTICAL_FLOW.flow_y       = 522;//optical_par.delta_Y; //	Flow in pixels * 10 in y-sensor direction (dezi-pixels)
//	OPTICAL_FLOW.flow_comp_m_x= Velocity.X.point; //Flow in meters in x-sensor direction, angular-speed compensated
//	OPTICAL_FLOW.flow_comp_m_y= Velocity.Y.point; //Flow in meters in y-sensor direction, angular-speed compensated
//	OPTICAL_FLOW.ground_distance= //	Ground distance in meters. Positive value: distance known. Negative value: Unknown distance
	OPTICAL_FLOW.quality=11;
//	OPTICAL_FLOW.time_usec=
  //*********************************************************************************************
//	BATT_STATUS.battery_function=MAV_BATTERY_FUNCTION_ALL;
//	BATT_STATUS.type       = MAV_BATTERY_TYPE_LIPO;
//	BATT_STATUS.voltages[1]= 10*(Bat_/BATT_Coef)/3;
//  BATT_STATUS.voltages[2]= 10*(Bat_/BATT_Coef)/3;
//  BATT_STATUS.voltages[3]= 10*(Bat_/BATT_Coef)/3;
//	BATT_STATUS.battery_remaining=100*((Bat_/BATT_Coef)/12.6);
//	BATT_STATUS.current_battery = 
//	BATT_STATUS.temperature=
//	BATT_STATUS.current_consumed=
//BATT_STATUS.energy_consumed =
  //***********************************************************************************************
//	ALTITUDE.altitude_local = Ultra.point ; 
//	ALTITUDE.altitude_monotonic = ALTITUDE_OFFSET;
//		ALTITUDE.altitude_amsl =Ultra.point;
//		ALTITUDE.altitude_relative=Ultra.point;
//		ALTITUDE.altitude_terrain = Ultra.point;
//		ALTITUDE.bottom_clearance = Ultra.point;
  //*********************************************************************************************
//    RC_RAW_PPM_CHANNELS.chan1_raw = RC.RC_channel[0];
//		RC_RAW_PPM_CHANNELS.chan2_raw = RC.RC_channel[1];
//		RC_RAW_PPM_CHANNELS.chan3_raw = RC.RC_channel[2];
//		RC_RAW_PPM_CHANNELS.chan4_raw = RC.RC_channel[3];
//		RC_RAW_PPM_CHANNELS.chan5_raw = RC.RC_channel[4];
//		RC_RAW_PPM_CHANNELS.chan6_raw = RC.RC_channel[5];
//		RC_RAW_PPM_CHANNELS.chan7_raw = RC.RC_channel[6];	 
//		RC_RAW_PPM_CHANNELS.chan8_raw = RC.RC_channel[7];	
//		RC_RAW_PPM_CHANNELS.chancount = 8;
//	RC_RAW_CHANNELS.rssi=
//	RC_RAW_CHANNELS.time_boot_ms = 
  //*********************************************************************************************
//		RC_CHANNELS_RAW.chan1_raw = RC.RC_channel[0];
//		RC_CHANNELS_RAW.chan2_raw = RC.RC_channel[1];
//		RC_CHANNELS_RAW.chan3_raw = RC.RC_channel[2];
//		RC_CHANNELS_RAW.chan4_raw = RC.RC_channel[3];
//		RC_CHANNELS_RAW.chan5_raw = RC.RC_channel[4];
//		RC_CHANNELS_RAW.chan6_raw = RC.RC_channel[5];
//		RC_CHANNELS_RAW.chan7_raw = RC.RC_channel[6];
//		RC_CHANNELS_RAW.chan8_raw = RC.RC_channel[7];
//  RC_CHANNELS_RAW.port      =
//	RC_CHANNELS_RAW.rssi      = 
//	RC_CHANNELS_RAW.time_boot_ms =
//***********************************************************************************************
//    RC_CHANNEL_SCALED.chan1_scaled = RC.RC_TRIM ;
//    RC_CHANNEL_SCALED.chan2_scaled = RC.HOV_THR ;
//	  RC_CHANNEL_SCALED.chan3_scaled = RC.RC_SW   ;
//		RC_CHANNEL_SCALED.chan4_scaled = RC.HOV_PIT ;
//		RC_CHANNEL_SCALED.chan5_scaled = RC.Yaw     ;
//		RC_CHANNEL_SCALED.chan6_scaled = RC.Roll    ;
//		RC_CHANNEL_SCALED.chan7_scaled = RC.Throttle;
//		RC_CHANNEL_SCALED.chan8_scaled = RC.Pitch   ;
//	RC_CHANNEL_SCALED.port         =
//	RC_CHANNEL_SCALED.rssi         =
//	RC_CHANNEL_SCALED.time_boot_ms =
  //**********************************************************************************************
		
	//payane meghdar dehi moteghayer haye mavlink*************************************************
	
	// Define the system type, in this case an airplane
	uint8_t system_type = MAV_TYPE_QUADROTOR;
	uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;
	 
	uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
	uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
	uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight
	 
	// Initialize the required buffers
	mavlink_message_t msg;
	
	mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &msg, system_type, autopilot_type, system_mode, custom_mode, system_state);
	// Copy the message to the send buffer
	len = mavlink_msg_to_send_buffer(buf, &msg);
	HAL_UART_Transmit(&huart1,(uint8_t*)buf,len,10);	
	
	// Pack the message
	mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &msg, system_type, autopilot_type, system_mode, custom_mode, system_state);
	// Copy the message to the send buffer
	len = mavlink_msg_to_send_buffer(buf, &msg);
	HAL_UART_Transmit(&huart4,(uint8_t*)buf,len,10);	
	//NrF_Fill_Data(&NRF,1,buf_2);	 CONTROL_SYSTEM_STATE
	//**************************************************************************************** 
//	mavlink_msg_highres_imu_encode(mavlink_system.sysid,mavlink_system.compid,&msg,&HIGHRES_IMU);
//	len = mavlink_msg_to_send_buffer(buf, &msg); 
//	HAL_UART_Transmit(&huart1,(uint8_t*)buf,len,10);
//	//**************************************************************************************** 
//	mavlink_msg_attitude_quaternion_encode(mavlink_system.sysid,mavlink_system.compid,&msg,&ATTITUDE_QUATERNION);
//	len = mavlink_msg_to_send_buffer(buf, &msg); 
//	HAL_UART_Transmit(&huart1,(uint8_t*)buf,len,10);
			//**************************************************************************************** 
//	mavlink_msg_scaled_imu_encode(mavlink_system.sysid,mavlink_system.compid,&msg,&SCALED_IMU);
//	len = mavlink_msg_to_send_buffer(buf, &msg); 
//	HAL_UART_Transmit(&huart1,(uint8_t*)buf,len,1);
//		//**************************************************************************************** 
//	mavlink_msg_local_position_ned_cov_encode(mavlink_system.sysid,mavlink_system.compid,&msg,&LOCAL_POSITION_NED_COV);
//	len = mavlink_msg_to_send_buffer(buf, &msg); 
//	HAL_UART_Transmit(&huart1,(uint8_t*)buf,len,1);
//	//**************************************************************************************** 
//	mavlink_msg_mission_count_encode(mavlink_system.sysid,mavlink_system.compid,&msg,&MISSION_COUNT);
//	len = mavlink_msg_to_send_buffer(buf, &msg); 
//	HAL_UART_Transmit(&huart1,(uint8_t*)buf,len,1);
//	//**************************************************************************************** 
//	mavlink_msg_mission_request_list_encode(mavlink_system.sysid,mavlink_system.compid,&msg,&MISSION_REQUEST_LIST);
//	len = mavlink_msg_to_send_buffer(buf, &msg); 
//	HAL_UART_Transmit(&huart1,(uint8_t*)buf,len,1);
//	//**************************************************************************************** 
//	mavlink_msg_mission_current_encode(mavlink_system.sysid,mavlink_system.compid,&msg,&MISSION_CURRENT);
//	len = mavlink_msg_to_send_buffer(buf, &msg); 
//	HAL_UART_Transmit(&huart1,(uint8_t*)buf,len,1);
//	//**************************************************************************************** 
//		 mavlink_msg_mission_set_current_encode(mavlink_system.sysid,mavlink_system.compid,&msg,&MISSION_SET_CURRENT);
//	 len = mavlink_msg_to_send_buffer(buf, &msg); 
//	 HAL_UART_Transmit(&huart1,(uint8_t*)buf,len,1);
//	//****************************************************************************************
//	 mavlink_msg_mission_request_encode(mavlink_system.sysid,mavlink_system.compid,&msg,&MISSION_REQUEST);
//	 len = mavlink_msg_to_send_buffer(buf, &msg); 
//	 HAL_UART_Transmit(&huart1,(uint8_t*)buf,len,1);	
//	 //****************************************************************************************
//	 mavlink_msg_mission_item_encode(mavlink_system.sysid,mavlink_system.compid,&msg,&MISSION_ITEM);
//	 len = mavlink_msg_to_send_buffer(buf, &msg); 
//	 HAL_UART_Transmit(&huart1,(uint8_t*)buf,len,1);	
//	 //****************************************************************************************
//	 mavlink_msg_manual_setpoint_encode(mavlink_system.sysid,mavlink_system.compid,&msg,&RC_SETPOINT);
//	 len = mavlink_msg_to_send_buffer(buf, &msg); 
//	 HAL_UART_Transmit(&huart1,(uint8_t*)buf,len,1);	
//	//********************************************************************************************
//	 mavlink_msg_sim_state_encode(mavlink_system.sysid,mavlink_system.compid,&msg,&SIM_STATE);
//	 len = mavlink_msg_to_send_buffer(buf, &msg); 
//	 HAL_UART_Transmit(&huart1,(uint8_t*)buf,len,1);	
	//********************************************************************************************
//	 mavlink_msg_attitude_encode(mavlink_system.sysid,mavlink_system.compid,&msg,&ATTITUDE);
//	 len = mavlink_msg_to_send_buffer(buf, &msg); 
//	 HAL_UART_Transmit(&huart1,(uint8_t*)buf,len,1);	
//	 //*******************************************************************************************
	 mavlink_msg_optical_flow_encode(mavlink_system.sysid,mavlink_system.compid,&msg,&OPTICAL_FLOW);
	 len = mavlink_msg_to_send_buffer(buf, &msg); 
	 HAL_UART_Transmit(&huart4,(uint8_t*)buf,len,1);	
//   //********************************************************************************************	 
//	 mavlink_msg_battery_status_encode(mavlink_system.sysid,mavlink_system.compid,&msg,&BATT_STATUS);
//	 len = mavlink_msg_to_send_buffer(buf, &msg); 
//	 HAL_UART_Transmit(&huart1,(uint8_t*)buf,len,1);	
//   //********************************************************************************************	 	 
//	 mavlink_msg_altitude_encode(mavlink_system.sysid,mavlink_system.compid,&msg,&ALTITUDE);
//	 len = mavlink_msg_to_send_buffer(buf, &msg); 
//	 HAL_UART_Transmit(&huart1,(uint8_t*)buf,len,1);	
//   //********************************************************************************************	 
//	 mavlink_msg_rc_channels_encode(mavlink_system.sysid,mavlink_system.compid,&msg,&RC_RAW_PPM_CHANNELS);
//	 len = mavlink_msg_to_send_buffer(buf, &msg); 
//	 HAL_UART_Transmit(&huart1,(uint8_t*)buf,len,1);	
//   //********************************************************************************************	 
//	 mavlink_msg_rc_channels_raw_encode(mavlink_system.sysid,mavlink_system.compid,&msg,&RC_CHANNELS_RAW);
//	 len = mavlink_msg_to_send_buffer(buf, &msg); 
//	 HAL_UART_Transmit(&huart1,(uint8_t*)buf,len,1);	
//   //********************************************************************************************	 
//	 mavlink_msg_rc_channels_scaled_encode(mavlink_system.sysid,mavlink_system.compid,&msg,&RC_CHANNEL_SCALED);
//	 len = mavlink_msg_to_send_buffer(buf, &msg); 
//	 HAL_UART_Transmit(&huart1,(uint8_t*)buf,len,1);	
//   //********************************************************************************************		 
//	 NrF_Fill_Data(&NRF,1,buf_2);
//	 
	// Send the message with the standard UART send function
	// uart0_send might be named differently depending on
	// the individual microcontroller / library in use.
	
}


void decode_mavlink_packet(uint8_t c,_MPC* Mpc)
{	

	// Try to get a new message
	
	
	if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
		//Mpc->ready=1;

			// Handle me,ssage
			switch(msg.msgid)
			{
				case MAVLINK_MSG_ID_HEARTBEAT:
				{	
							
					mavlink_heartbeat_t data;
						mavlink_msg_heartbeat_decode(&msg,&data);			
									
				}	
				break;
//				case MAV_CMD_DO_CHANGE_ALTITUDE :
					
			case MAVLINK_MSG_ID_COMMAND_LONG:					
				break;
			case MAVLINK_MSG_ID_OPTICAL_FLOW:
			{
				decode_flag_mavlink_usart++;	

				mavlink_msg_optical_flow_decode(&msg,&OPTICAL_FLOW);
			  mpc_data[0]= OPTICAL_FLOW.flow_x;  
		   	mpc_data[1]= OPTICAL_FLOW.flow_y;  
		  	quality= OPTICAL_FLOW.quality;
			//		Mpc->ready=0;
	} 
			
	   	break;
			
			
			
			default:				
					break;
			}
	}
}



void decode_mavlink_packet_2(uint8_t c,_MPC* Mpc)
{	

	// Try to get a new message
	
	
	if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
		//Mpc->ready=1;

			// Handle me,ssage
			switch(msg.msgid)
			{
				case MAVLINK_MSG_ID_HEARTBEAT:
				{	
							decode_flag_mavlink_usart++;	

					mavlink_heartbeat_t data;
						mavlink_msg_heartbeat_decode(&msg,&data);			
									
				}	
				break;
//				case MAV_CMD_DO_CHANGE_ALTITUDE :
					
			case MAVLINK_MSG_ID_COMMAND_LONG:					
				break;
			case MAVLINK_MSG_ID_OPTICAL_FLOW:
			{
				mavlink_msg_optical_flow_decode(&msg,&OPTICAL_FLOW);
			  mpc_data_2[0]= OPTICAL_FLOW.flow_x;  
		   	mpc_data_2[1]= OPTICAL_FLOW.flow_y;  
		  	quality= OPTICAL_FLOW.quality;
			//		Mpc->ready=0;
	} 
			
	   	break;
			
			
			
			default:				
					break;
			}
	}
}
