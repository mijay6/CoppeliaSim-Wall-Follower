from enum import IntEnum
from dataclasses import dataclass
import time
import coppelia_client
import signal

# Configuration
@dataclass
class Config:
    base_speed: float = 0.9

    wall_dist: float = 0.25
    max_detection_dist: float = 1.0
    wall_lost_time: float = 1.0

    emergency_dist: float = 0.13
    front_turn_dist: float = 0.36
    
    kp: float = 14.0
    ki: float = 0.2
    kd: float = 14.0

    avoid_gain: float = 5.0

    max_turn: float = 0.8
    integral_limit: float = 0.4

    smoothing_alpha: float = 0.5

    debug: bool = True
    debug_query: float = 0.25

class RobotState(IntEnum):
    SEARCH_WALL = 0
    FOLLOW_WALL = 1

class Side(IntEnum):
    RIGHT = 1
    LEFT = -1

# Sensor processing
@dataclass
class SensorData:
    front_left: float
    front_center: float
    front_right: float
    back_left: float
    back_center: float
    back_right: float

    def get_front_side(self, side: Side) -> float:
        return self.front_right if side == Side.RIGHT else self.front_left
    
    def get_back_side(self, side: Side) -> float:
        return self.back_right if side == Side.RIGHT else self.back_left

class SensorProcessor:
    @staticmethod
    def process(readings: list) -> SensorData:
        front_left = min(readings[0], readings[1], readings[2])
        front_center = min(readings[3], readings[4])
        front_right = min(readings[5], readings[6], readings[7])

        back_left = min(readings[13], readings[14], readings[15])
        back_center = min(readings[11], readings[12])
        back_right = min(readings[8], readings[9], readings[10])

        return SensorData(front_left, front_center, front_right, back_left, back_center, back_right)


# PID helper
class PIDController:
    def __init__(self, kp: float, ki: float, kd: float, integral_limit: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()

    def compute(self, error: float, current_time: float) -> float:
        dt = current_time - self.last_time
        if dt <= 0:
            dt: 0.05

        p_term = self.kp * error
        
        self.integral = error * dt
        self.integral = max(min(self.integral, self.integral_limit),-self.integral_limit)
        i_term = self.ki * self.integral

        d_term = self.kd * (error - self.prev_error) / dt
 
        self.prev_error = error
        self.last_time = current_time

        return p_term + i_term + d_term

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()

# Wall follower
class WallFollower:
    def __init__(self, config: Config = None):
        self.config = config or Config()

        self.pid = PIDController(self.config.kp, self.config.ki, self.config.kd, self.config.integral_limit)

        self.state = RobotState.SEARCH_WALL
        self.last_wall_seen = time.time()
        self.last_debug_time = 0.0
        self.prev_turn = 0.0
    
    def update(self, robot) -> tuple:
        now = time.time()
        sensors = SensorProcessor.process(robot.get_sonar())

        if self.state == RobotState.SEARCH_WALL:
            return self._search(sensors,now)
        return self._follow(sensors,now)

    def _search(self, sensors: SensorData, now: float) -> tuple:
        threshold = self.config.wall_dist * 1.5
        self._log("Searching Wall")

        if sensors.front_right < threshold and sensors.front_left > sensors.front_right * 0.9:
            self._set_follow_side(Side.RIGHT, now)
            self._log("Wall on RIGHT detected")
            return 0.0, 0.0

        if sensors.front_left < threshold and sensors.front_right > sensors.front_left * 0.9:
            self._set_follow_side(Side.LEFT, now)
            self._log("Wall on LEFT detected")
            return 0.0, 0.0

        if sensors.front_center < threshold:
            if sensors.front_left > sensors.front_right:
                self._log("Turning right")
                return self.config.base_speed * 0.4, -self.config.base_speed * 0.4
            self._log("Turning left")
            return -self.config.base_speed * 0.4, self.config.base_speed * 0.4
        
        return self.config.base_speed, self.config.base_speed

    def _follow(self, sensors: SensorData, now: float) -> tuple:
        front_side = sensors.get_front_side(self.follow_side)
        back_side = sensors.get_back_side(self.follow_side)

        wall_follow_dist = self.config.wall_dist * 2.5
        if front_side < wall_follow_dist or back_side < wall_follow_dist:
            self.last_wall_seen = now

        if (now - self.last_wall_seen) > self.config.wall_lost_time:
            self._log("Wall lost")
            self.state = RobotState.SEARCH_WALL
            return self.config.base_speed, self.config.base_speed

        min_front = min(sensors.front_left, sensors.front_center, sensors.front_right)
        if min_front < self.config.emergency_dist:
            self._log("Emergency front, reversing")
            self._reset_controller(now)
            return self._reverse_and_turn()
        
        min_back = min(sensors.back_left, sensors.back_center, sensors.back_right)
        if min_back < self.config.emergency_dist:
            self._log("Emergency rear, advancing")
            self._reset_controller(now)
            return self._advance_and_turn()
        
        if sensors.front_center < self.config.front_turn_dist:
            return self._turn_outward(sensors)

        return self._pid_control(sensors, now)


    def _pid_control(self, sensors: SensorData, now: float) -> tuple:
        front_side = sensors.get_front_side(self.follow_side)
        back_side = sensors.get_back_side(self.follow_side)
        opposite_front = sensors.get_front_side(Side.LEFT if self.follow_side == Side.RIGHT else Side.RIGHT)

        dist_error = front_side - self.config.wall_dist
        dist_turn = self.pid.compute(dist_error,now) * -self.follow_side.value

        avoid_error =  max(0.0, (self.config.wall_dist * 1.2) - opposite_front)
        avoid_turn = avoid_error * self.config.avoid_gain * -self.follow_side.value

        turn = dist_turn + avoid_turn
        max_turn_effect = self.config.max_turn * self.config.base_speed
        turn = max(-max_turn_effect, min(max_turn_effect, turn))

        turn = self.config.smoothing_alpha * turn + (1 - self.config.smoothing_alpha) * self.prev_turn
        turn = max(-max_turn_effect, min(max_turn_effect, turn))
        self.prev_turn = turn

        speed = self.config.base_speed
        if front_side < self.config.wall_dist * 0.8:
            speed *= 0.80

        left_speed = speed - turn 
        right_speed = speed + turn

        self._debug_pid(front_side, back_side, dist_error, turn, now)
        
        return left_speed, right_speed

    def _turn_outward(self, sensors: SensorData) -> tuple:
        speed = self.config.base_speed * 0.5
        if self.follow_side == Side.RIGHT:
            return -speed, speed
        return speed, -speed

    def _reverse_and_turn(self) -> tuple:
        speed = self.config.base_speed
        if self.follow_side == Side.RIGHT:
            return -speed, -speed * 0.2  
        return -speed * 0.2, -speed 

    def _advance_and_turn(self) -> tuple:
        speed = self.config.base_speed
        if self.follow_side == Side.RIGHT:
            return speed * 0.2, speed
        return speed, speed * 0.2

    def _set_follow_side(self, side: Side, now: float):
        self.follow_side = side
        self.state = RobotState.FOLLOW_WALL
        self.last_wall_seen = now
        self._reset_controller(now)

    def _reset_controller(self, now: float):
        self.pid.reset()
        self.pid.last_time = now
    
    def _log(self, message: str):
        if self.config.debug:
            print(message)

    def _debug_pid(
        self,
        front_side: float,
        back_side: float,
        dist_error: float,
        turn: float,
        now: float,
    ):
        if not self.config.debug: return
        if (now - self.last_debug_time) < self.config.debug_query: return
        self.last_debug_time = now
        side_name = self.follow_side.name if self.follow_side else "NONE"
        print( f"[PID {side_name}] front_dist={front_side:.3f}m back_dist={back_side:.3f}m d_err={dist_error:+.3f} turn={turn:+.3f}" )

# Entrypoint
def main(args=None):  
    coppelia = coppelia_client.Coppelia()
    robot = coppelia_client.P3DX(coppelia.sim, "PioneerP3DX")
    config = Config()
    controller = WallFollower(config)

    # -------
    # Corected code for shortCut Ctrl + C 
    # -------
    # Imported signal module at the top

    running = True
    def on_ctrl_c(sig, frame):
        nonlocal running
        running = False # change the running flag to False to exit the loop while

    signal.signal(signal.SIGINT, on_ctrl_c) # When Ctrl+C is pressed call on_ctrl_c

    coppelia.start_simulation()
    print("Starting wall follower PID")

    while running and coppelia.is_running():
        left, right = controller.update(robot)
        robot.set_speed(left, right)
        time.sleep(0.05)

    robot.set_speed(0, 0)
    coppelia.stop_simulation()
    print("Simulation stopped")

if __name__ == "__main__":
    main()