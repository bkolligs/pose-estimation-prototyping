#ifndef _POSE_EVENTS_H_
#define _POSE_EVENTS_H_

// CFE related
#define CFE_SUCCESS 1

// various pose estimator events
#define POSE_RESERVED_EID                     0
#define POSE_STARTUP_INF_EID                  1
#define POSE_COMMAND_ERR_EID                  2
#define POSE_COMMANDNOP_INF_EID               3
#define POSE_COMMANDRST_INF_EID               4
#define POSE_INVALID_MSGID_ERR_EID            5
#define POSE_LEN_ERR_EID                      6
#define POSE_PIPE_ERR_EID                     7
#define POSE_START_INF_EID                    8
#define POSE_STOP_INF_EID                     9
#define POSE_EKF_IMU_INITDATA_ERR_EID         10
#define POSE_EKF_MALFORMED_ROT_MATRIX_ERR_EID 11
#define POSE_EKF_COMPUTE_PHI_QDK_ERR_EID      12
#define POSE_EKF_SENTPOSE_INF_EID             13

#define POSE_EVENT_COUNTS 13

#endif //_POSE_EVENTS_H_ header