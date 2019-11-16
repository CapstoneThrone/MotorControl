# MotorControl
Aruino Code for Motor Controller


 * First 4 bytes are left, right, forward, and backward
 * 1000 <- left 0100 <- right 0010 <- forward 0001 <- backward
 * 
 * Last 4 bytes are a multiplexed movement speed
 * 
 * Examples: 0100 1111 would mean turn right at 100% movement speed,
 *           0001 1000 would mean drive backward at 50% movement speed
