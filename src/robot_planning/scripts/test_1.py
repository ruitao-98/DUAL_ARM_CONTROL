#! /home/yanji/anaconda3/envs/mujo/bin/python3
# from pynput.keyboard import Key, Listener

# def wait_for_key_press():
#     key_pressed = {'value': None}

#     def on_press(key):
#         if key == Key.backspace:
#             print("Backspace pressed, returning 0.")
#             key_pressed['value'] = 0
#             return False  # Stop the listener
#         elif key == Key.enter:
#             print("Enter pressed, returning 1.")
#             key_pressed['value'] = 1
#             return False  # Stop the listener

#     with Listener(on_press=on_press) as listener:
#         print("please press 'enter' to ensure the trajectory or press 'backspace' to replan" )
#         listener.join()
    
#     return key_pressed['value']

# # 使用 wait_for_key_press 函数等待按键并获取返回值
# result = wait_for_key_press()
# print(f"Function returned: {result}")

def aa():
    print ( "a")

# class RobotKdl():

#     def __init__(self):
#         self.a = 1
        

#     def aa(self):
#         print ( self.a)
