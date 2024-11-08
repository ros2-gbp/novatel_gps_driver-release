# Based on https://docs.novatel.com/OEM7/Content/Logs/RXSTATUS.htm
std_msgs/Header header

NovatelMessageHeader novatel_msg_header

# for the bit table of the following values see https://docs.novatel.com/OEM7/Content/Logs/RXSTATUS.htm
uint32 error

uint32 rxstat

uint32 aux1stat

uint32 aux2stat

uint32 aux3stat

uint32 aux4stat
