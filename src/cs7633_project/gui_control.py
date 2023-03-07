from cs7633_project.robot_control import DriveControlAction, ManipulationControlAction

class GUICallbacks:
    def manipulation_action(action_name: str) -> ManipulationControlAction:
        return ManipulationControlAction.make(action_name)

    def drive_action(action_name: str) -> DriveControlAction:
        return DriveControlAction.make(action_name)
