
from typing import List, Tuple, Union
class Robot:
    """
    Robot class to store and ID all new robots currently being tracked.

    Args:
        None
    """

    def __init__(self):
        self.velocity_list = []  # stores bot velocities per frame
        self.position_list = []  # stores bot positions per frame
        self.blur_list = []  # stores calculated blur values per frame (AKA z-value)
        self.frame_list = []  # unused? stores frames
        self.area_list = []  # stores the cropped areas
        self.cropped_frame = []  # cropped section of a frame representing the bot
        self.avg_area = 0  # current average area of the bot in this frame
        self.trajectory = []  # track points from manual pathing
        self.times = []  #time step per frame in seconds
        self.stuck_status_list = [] #whether or not the robot is stuck or not
        self.crop_length = 40
        

    def add_area(self, area: float):
        self.area_list.append(area)

    def add_blur(self, blur: float):
        self.blur_list.append(blur)

    def add_velocity(self, velocity):
        self.velocity_list.append(velocity)

    def add_position(self, position: List[float]):
        self.position_list.append(position)
    
    def add_frame(self, frame: int):
        self.frame_list.append(frame)

    def add_crop(self, crop: List[int]):
        self.cropped_frame.append(crop)

    def set_avg_area(self, avg_area: float):
        self.avg_area = avg_area

    def add_trajectory(self, traj):
        self.trajectory.append(traj)

    def add_time(self, time):
        self.times.append(time)

    def add_stuck_status(self,stuck):
        self.stuck_status_list.append(stuck)
    

    def as_dict(self) -> dict:
        """
        Convert's the bot's current frame and position information into a
        readable dictionary

        Args:
            None

        Returns:
            dictionary of the bot's current position and frame information
        """
        mydict = {
            "Frame": self.frame_list,
            "Times": self.times,
            "Position": self.position_list,
            "Velocity": self.velocity_list,
            "Cropped Frame Dim": self.cropped_frame,
            "Area": self.area_list,
            "Blur": self.blur_list,
            "Avg Area": self.avg_area,
            "Trajectory": self.trajectory,
        }

        return mydict
