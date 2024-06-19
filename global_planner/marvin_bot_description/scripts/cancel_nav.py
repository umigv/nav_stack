from robot_navigator import BasicNavigator, NavigationResult
import rclpy

rclpy.init()
nav = BasicNavigator()

nav.waitUntilNav2Active() #

print('CANCELING THE NAV CANCELING THE NAV')
print('CANCELING THE NAV CANCELING THE NAV')
print('CANCELING THE NAV CANCELING THE NAV')
button_pressed = True
if(button_pressed):
    nav.cancelTask()
    

