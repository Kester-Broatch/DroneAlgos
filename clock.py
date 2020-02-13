#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''-------------------------- Program Header ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------''
  ' Add all important details regarding the program here
  '
  ' Project             : BAE Swarm Challenge - Group Design Project
  ' Program name        : SAMPLE_CODE_gdpa.py
  ' Author              : Romain Delabeye
  ' Adm. structure      : Cranfield University
  ' Date (YYYYMMDD)     : 20200205
  ' Purpose             : provide code structure
  ' User                : A_Team
  '
  ' Revision History    : 1.0
  '
  ' Date YYYYMMDD |  Author          | Ref       |Revision comment
  '-------------------------------------------------------------------
  ' 20200205      |  Romain DELABEYE |           | Developing & Structuring - howto publish&subscribe; struct
  '               |                  |           | 
  '               |                  |           | 
  '               |                  |           | 


TODO:
""" Put here all your tasks """
(You can also add TODO anywhere in the code if you need to come back to a certain line later on)

-
-
-

''-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------'''
'''
'------------------------------------------------------------------------------------------------------------------------
'    import libraries
'------------------------------------------------------------------------------------------------------------------------
'''

#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64

def gettime():
    pub = rospy.Publisher('time', Float64, queue_size=10)
    rospy.init_node('clock', anonymous=False)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
         time = rospy.get_time()
         #rospy.loginfo(time)
         pub.publish(time)
         rate.sleep()
 
if __name__ == '__main__':
     try:
         gettime()
     except rospy.ROSInterruptException:
         pass