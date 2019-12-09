# Xair-footswitch

with this you can command your XAir console (in my case a XR16) with a 3 button footswitch  non latching (temporary) and a raspberry. Sends OSC commands over ethernet. 


button 1 & 2 command the mutes from fx 1 to fx 4 and volume channel
button 3 command the tap delay of fx1

button 1 short push = fx1 button 1 long push = fx2
button 2 short push = fx3 button 2 long push = fx4

button 1 & 2 : very long push = increase/decrease volume on aux master 2

code is commented so should be ok to adapt to any commands you need. Just have a look to the array containing the address/string

connection errors are managed including self-discovery of the console IP (auto connect)

Enjoy
