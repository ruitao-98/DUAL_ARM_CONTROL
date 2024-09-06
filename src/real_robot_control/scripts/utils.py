from scipy.spatial.transform import Rotation as R
from pynput.keyboard import Key, Listener

def from_eular_to_matrix(rpy_temp):
    """
    rpy_temp is a list, with three components, rx, ry, rz
    """
    rotation_z = R.from_euler('z', rpy_temp[2], degrees=False)
    rotation_y = R.from_euler('y', rpy_temp[1], degrees=False)
    rotation_x = R.from_euler('x', rpy_temp[0], degrees=False)
    # 将旋转矩阵组合起来
    r_rotm = rotation_z * rotation_y * rotation_x
    # 提取旋转矩阵
    r_rotm_matrix = r_rotm.as_matrix()

    return r_rotm_matrix

def from_matrix_to_eular(r_rotm):
    r = R.from_matrix(r_rotm)
    euler_angles = r.as_euler('xyz', degrees=False)  # 使用 'xyz' 顺序，如果需要其他顺序可以调整
    rpy_temp_trans = euler_angles.tolist()

    """
    rpy_temp_trans is a list, with three components, rx, ry, rz
    """

    return rpy_temp_trans

def wait_for_key_press():
    key_pressed = {'value': None}

    def on_press(key):
        if key == Key.backspace:
            print("Backspace pressed, returning 0.")
            key_pressed['value'] = 0
            return False  # Stop the listener
        elif key == Key.enter:
            print("Enter pressed, returning 1.")
            key_pressed['value'] = 1
            return False  # Stop the listener

    with Listener(on_press=on_press) as listener:
        print("please press 'enter' to ensure the registration or press 'backspace' to re-regis" )
        listener.join()
    
    return key_pressed['value']

def wait_for_choice():
    key_pressed = {'value': None}

    def on_press(key):
        try:
            if key.char in ['0', '1', '2', '3', '4']:
                print(f"Key {key.char} pressed, returning {key.char}.")
                key_pressed['value'] = int(key.char)
                return False  # Stop the listener
        except AttributeError:
            pass  # Ignore non-character keys

    with Listener(on_press=on_press) as listener:
        print("please press '0-4' to select a choice" )
        listener.join()
    
    return key_pressed['value']
        