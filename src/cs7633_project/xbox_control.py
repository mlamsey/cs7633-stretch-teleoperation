from cs7633_project.xbox_controller import XboxController
from cs7633_project.robot_control import ManipulationControlAction, DriveControlAction, ControllerAction
from cs7633_project.srv import ControlActionRequest

############################################################
def _manipulation_action_left_stick(gamepad_state: dict) -> ManipulationControlAction:
    x = gamepad_state["left_stick_x"]
    y = gamepad_state["left_stick_y"]
    if x > 0.5:
        return ManipulationControlAction.RIGHT
    elif x < -0.5:
        return ManipulationControlAction.LEFT
    
    if y > 0.5:
        return ManipulationControlAction.FORWARD
    elif y < -0.5:
        return ManipulationControlAction.BACKWARD

    return ManipulationControlAction.IDLE

def _manipulation_action_right_stick(gamepad_state: dict) -> ManipulationControlAction:
    x = gamepad_state["right_stick_x"]
    y = gamepad_state["right_stick_y"]
    
    if y > 0.5:
        return ManipulationControlAction.UP
    elif y < -0.5:
        return ManipulationControlAction.DOWN

    return ManipulationControlAction.IDLE

def _manipulation_action_buttons(gamepad_state: dict) -> ManipulationControlAction:
    a = gamepad_state["bottom_button_pressed"]
    b = gamepad_state["right_button_pressed"]
    x = gamepad_state["left_button_pressed"]
    y = gamepad_state["top_button_pressed"]

    if a:
        return ManipulationControlAction.GRASP
    elif b:
        return ManipulationControlAction.RELEASE
    elif y:
        return ControllerAction.CHANGE_MODE

    return ManipulationControlAction.IDLE

def _drive_action_left_stick(gamepad_state: dict):
    x = gamepad_state["left_stick_x"]
    y = gamepad_state["left_stick_y"]
    if x > 0.5:
        return DriveControlAction.TURN_CW
    elif x < -0.5:
        return DriveControlAction.TURN_CCW
    
    if y > 0.5:
        return DriveControlAction.FORWARD
    elif y < -0.5:
        return DriveControlAction.BACKWARD
    
    return DriveControlAction.IDLE

def _drive_action_right_stick(gamepad_state: dict):
    return DriveControlAction.IDLE

def _drive_action_buttons(gamepad_state: dict):
    y = gamepad_state["top_button_pressed"]
    if y:
        return ControllerAction.CHANGE_MODE
    else:
        return DriveControlAction.IDLE

############################################################
class XboxControl:
    def __init__(self) -> None:
        self.gamepad = XboxController()
        self.gamepad.start()

    def _get_controller_state(self):
        return self.gamepad.get_state()
    
    def _get_manipulation_action(self):
        # get state
        gamepad_state = self._get_controller_state()
        l_stick_action = _manipulation_action_left_stick(gamepad_state)
        r_stick_action = _manipulation_action_right_stick(gamepad_state)
        button_action = _manipulation_action_buttons(gamepad_state)

        # return
        if l_stick_action != ManipulationControlAction.IDLE:
            return l_stick_action
        
        if r_stick_action != ManipulationControlAction.IDLE:
            return r_stick_action
        
        if button_action != ManipulationControlAction.IDLE:
            return button_action

        return ManipulationControlAction.IDLE
    
    def _get_drive_action(self):
        # get state
        gamepad_state = self._get_controller_state()
        l_stick_action = _drive_action_left_stick(gamepad_state)
        r_stick_action = _drive_action_right_stick(gamepad_state)
        button_action = _drive_action_buttons(gamepad_state)

        # return
        if l_stick_action != DriveControlAction.IDLE:
            return l_stick_action
        
        if r_stick_action != DriveControlAction.IDLE:
            return r_stick_action
        
        if button_action != DriveControlAction.IDLE:
            return button_action

        return DriveControlAction.IDLE

    def get_action(self, mode: ControlActionRequest=ControlActionRequest.CONTROLLER_MANIPULATION):
        if mode == ControlActionRequest.CONTROLLER_MANIPULATION:
            return self._get_manipulation_action()
        elif mode == ControlActionRequest.CONTROLLER_DRIVE:
            return self._get_drive_action()
        else:
            print("XboxControl::get_action: warning! invalid mode! returning None")
            return None