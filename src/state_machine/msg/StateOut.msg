uint8 State_Idle=0
uint8 State_P2P=1
uint8 State_Identify=2
uint8 State_D_Approach=3
uint8 State_U_Approach=4
uint8 State_Verify=5
uint8 State_Retrace=6
uint8 State_Lock=7
uint8 State_Unlock=8
uint8 State_EHS= 9
uint8 OperationMode_Standby = 0
uint8 OperationMode_Pickup = 1
uint8 OperationMode_DropOff = 2
Header header 
string Diagnostics
uint8 PodInfo
uint8 HMSCheck
uint8 OperationMode
uint8 PrevState
uint8 CurrState