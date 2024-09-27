

class print_data():
    def __init__(self):
        self.boat_x = 0
        self.boat_y  = 0
        self.state = 0
        self.target = 0
        self.error_angle = 0
        self.mark_area = 0
        self.mart_detect_area = 0
        self.target_detect_area = 0
        self.arrival_target_area = 0
        self.psi_desire = 0
        self.psi  = 0
        self.target_shape = 0
        self.target_color = 0
        self.mark_check_cnt =0  
        self.target_detect_time = 0
        self.detected_cnt = 0 
        self.target_detect_cnt = 0
        self.mark_detect_area = 0
        self.square = 0 
        self.triangle = 0
        self.circle = 0  
        self.cross = 0
        self.print_target_color = 0
        self.print_target_std = 0 
        self.thruster_speed_L = 0
        self.thruster_speed_R = 0
        self.color_range = 0
        self.distance_to_point = 0
        self.psi_goal = 0
        self.stop_cnt =0
        self.stop_time = 0

        self.inrange_obstacles = 0
        self.obstacles = 0
        pass
    
    def print_autonomous(self):
        error_angle = self.error_angle
        """print current state

        Args:
            error_angle (float) : angle between psi_desire and psi (heading to desire angle)
            u_servo (int) : servo moter publish value
        """
        print("")
        print("({:>4.2f}, {:>4.2f})".format(self.boat_x, self.boat_y))
        print("Obstacle  : {:2d} / {:2d}".format(len(self.inrange_obstacles), len(self.obstacles)))

        psi_goal_dir_str = "[   | * ]" if self.psi_goal > 0 else "[ * |   ]"
        error_angle_dir_str = "( Right )" if error_angle > 0 else "(  Left )"

        print("")
        print("{:^9}   {:^6} - {:^6} = {:^6} {:->9} {:^5}".format("goal", "desire", "psi", "error", ">", "servo"))
        print(
            "{:>9}   {:>6.2f} - {:>6.2f} = {:>6.2f} {:>9}".format(
                psi_goal_dir_str,
                self.psi_desire,
                self.psi,
                error_angle,
                error_angle_dir_str,

            )
        )
        print("")
        print("{:<9} : {:6.2f} m".format("distance", self.distance_to_goal))
        print("")
        print("-" * 70)        



    def print_dock(self):
        error_angle = self.error_angle
        

        state_str = [
            "Avoiding Obstacles",
            "Going to Station #1",
            "Going to Station #2",
            "Going to Station #3",
            "Rotating Heading",
            "Detecting Target",
            "Docking",
            "End",
        ]
        print("")
        print("({:>4.2f}, {:>4.2f})".format(self.boat_x, self.boat_y))
        print("State: # {} - {}  angle = {}".format(str(self.state), state_str[self.state] , error_angle))
        print("")

        if self.state == 6:
            error_angle_dir_str = "( Right )" if error_angle > 0 else "(  Left )"
            print("Mark Area    : {:>7,.0f} / {:>7,.0f}".format(self.mark_area, self.mark_detect_area))
            print(
                "Target Area  : {:>7,.0f} / {:>7,.0f} ({:>5})".format(
                    self.target[0] if len(self.target) != 0 else 0,
                    self.target_detect_area,
                    "Found" if len(self.target) != 0 else "None",
                )
            )
            print(
                "Arrival Area : {:>7,.0f} / {:>7,.0f}".format(
                    self.target[0] if len(self.target) != 0 else 0,
                    self.arrival_target_area,
                )
            )
            print("")
            print("mid - {:>6} = {:>11} {:->4} {:>11}".format("target", "error_pixel", ">", "error_angle"))
            print(
                "320 - {:>6,.0f} = {:>11,.0f} {:>4} {:>11.2f} {:>9}".format(
                    self.target[1] if len(self.target) != 0 else 0,
                    320 - self.target[1] if len(self.target) != 0 else 0,
                    "",
                    error_angle,
                    error_angle_dir_str,
                )
            )
            print("")

        if self.state in [0, 1, 2, 3]:
            psi_goal_dir_str = "[   | * ]" if self.psi_goal > 0 else "[ * |   ]"
            error_angle_dir_str = "( Right )" if error_angle > 0 else "(  Left )"
            print("{:^9}   {:^8} - {:^8} = {:^8} {:->9} {:^5}".format("goal", "desire", "psi", "error", ">", "servo"))
            print(
                "{:>9}   {:>8.2f} - {:>8.2f} = {:>8.2f} {:>9} ".format(
                    psi_goal_dir_str,
                    self.psi_desire,
                    self.psi,
                    error_angle,
                    error_angle_dir_str,
                )
            )
            print("")
            print("{:<9} : {:6.2f} m".format("distance", self.distance_to_point))

        elif self.state == 4:
            error_angle_dir_str = "( Right )" if error_angle > 0 else "(  Left )"

            if self.stop_cnt >= self.stop_time:
                print("Rotating Heading >>>>")
            else:
                print("Stopping Boat >>>>>>> {:>2d} / {:>2d}".format(self.stop_cnt, self.stop_time))
            print("")
            print("{:^8} - {:^8} = {:^8} {:->9} {:^5}".format("desire", "psi", "error", ">", "servo"))
            print(
                "{:>8.2f} - {:>8.2f} = {:>8.2f} {:>9}".format(
                    self.psi_desire,
                    self.psi,
                    error_angle,
                    error_angle_dir_str,
                )
            )

        elif self.state == 5:
            print("Target Shape : {} | Color : {}".format(self.target_shape, self.target_color))
            print("Waiting..... : {:>4d} / {:>4d}".format(self.mark_check_cnt, self.target_detect_time))
            print("Target Cnt   : {:>4d} / {:>4d}".format(self.detected_cnt, self.target_detect_cnt))
            print("")
            print("Mark Area    : {:>7,.0f} / {:>7,.0f}".format(self.mark_area, self.mark_detect_area))

        print("")
        print("ThrusterL  : {}".format(self.thruster_speed_L))
        print("ThrusterR  : {}".format(self.thruster_speed_R))

        print("")
        try:
            print("color : {}".format(self.color_check))
            print("square: {}, triangle: {}, circle {},  cross {}".format(self.square, self.triangle, self.circle,  self.cross))
            print("target: {}".format(self.target_shape))
            print("target_color: {}, std_color {}".format(self.print_target_color , self.print_target_std))
            print("color range {} , {}".format(self.color_range[0], self.color_range[1]))
        except:
            pass
        print("\n\n\n\n")

        print("-" * 70)