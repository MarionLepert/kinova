#pragma once

#include <string>

// BOT1
const std::string REDIS_KEY_BOT1_Q = "kinova::bot1::q";         // Robot joint angles
const std::string REDIS_KEY_BOT1_DQ = "kinova::bot1::dq";       // Robot joint velocities
const std::string REDIS_KEY_BOT1_Q_DES = "kinova::bot1::q_des";    // Robot joint angles command
const std::string REDIS_KEY_BOT1_DQ_DES = "kinova::bot1::dq_des";  // Robot joint velocities command

const std::string REDIS_KEY_BOT1_EE_POS = "kinova::bot1::ee_pos";         // Robot end-effector pos
const std::string REDIS_KEY_BOT1_EE_QUAT_WXYZ = "kinova::bot1::ee_quat_wxyz"; // Robot end-effector orientation
const std::string REDIS_KEY_BOT1_EE_POS_DES = "kinova::bot1::ee_pos_des";         // Robot end-effector pos command
const std::string REDIS_KEY_BOT1_EE_QUAT_WXYZ_DES = "kinova::bot1::ee_quat_wxyz_des"; // Robot end-effector orientation command

const std::string REDIS_KEY_BOT1_GRIPPER_POS = "kinova::bot1::gripper_position"; // Robot gripper position
const std::string REDIS_KEY_BOT1_GRIPPER_POS_DES = "kinova::bot1::gripper_position_des"; // Robot gripper position command

const std::string REDIS_KEY_BOT1_STATUS = "kinova::bot1::status";

// BOT2
const std::string REDIS_KEY_BOT2_Q = "kinova::bot2::q";         // Robot joint angles
const std::string REDIS_KEY_BOT2_DQ = "kinova::bot2::dq";       // Robot joint velocities
const std::string REDIS_KEY_BOT2_Q_DES = "kinova::bot2::q_des";    // Robot joint angles command
const std::string REDIS_KEY_BOT2_DQ_DES = "kinova::bot2::dq_des";  // Robot joint velocities command

const std::string REDIS_KEY_BOT2_EE_POS = "kinova::bot2::ee_pos";         // Robot end-effector pos
const std::string REDIS_KEY_BOT2_EE_QUAT_WXYZ = "kinova::bot2::ee_quat_wxyz"; // Robot end-effector orientation
const std::string REDIS_KEY_BOT2_EE_POS_DES = "kinova::bot2::ee_pos_des";         // Robot end-effector pos command
const std::string REDIS_KEY_BOT2_EE_QUAT_WXYZ_DES = "kinova::bot2::ee_quat_wxyz_des"; // Robot end-effector orientation command

const std::string REDIS_KEY_BOT2_GRIPPER_POS = "kinova::bot2::gripper_position"; // Robot gripper position
const std::string REDIS_KEY_BOT2_GRIPPER_POS_DES = "kinova::bot2::gripper_position_des"; // Robot gripper position command

const std::string REDIS_KEY_BOT2_STATUS = "kinova::bot2::status";



