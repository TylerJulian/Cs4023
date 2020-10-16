class PlanInput:

    EXAMPLE_TASKS = [((2, 2.5), (12.5, 2.5)), ((12.5, 12.5), (5, 12.5)), ((2,3),(9,8))]

    def __init__(self):
        pass

    @staticmethod
    def initial_user_prompt():
        print("Welcome, Choose one of following options or Ctrl+C to exit anytime: ")
        print("[1]: Enter coordinates for tasks by hand.")
        print("[2]: Use provided example set of tasks.")
        try:
            user_choice = raw_input()
        except KeyboardInterrupt:
            print("Exiting...")
            return -2
        if user_choice.lower() == '1':
            return 1
        elif user_choice.lower() == '2':
            return 2
        else:
            return -1

    @staticmethod
    def get_coordinates_from_user():
        print("Please enter a set of coordinates in the following format: ")
        print("((x1,y2),(x2,y2))")
        print("Or enter the START to start")
        # create a list of tasks to return
        task_list = []
        while True:
            is_start, u_in = PlanInput.get_input()
            if not is_start:

                try:
                    journey = PlanInput.parse_point(u_in)
                    task_list.append(journey)
                    print("Task: " + str(journey) + ", added, enter other point or START:")
                except ValueError:
                    print ("Wrong Format! enter other point or START:")
            else:
                break
        return task_list



    @staticmethod
    def get_input():
        try:
            u_in = raw_input()
        except KeyboardInterrupt:
            print("Exiting...")
            return False, None
        if u_in.lower() == "start":
            return True, u_in
        else:
            return False, u_in



    @staticmethod
    def parse_point(coordinate):
        """
        Given a journey string in the following format:
        ((x1,y2),(x2,y2))
        turns it to a tuple of two tuples

        @type coordinate: str
        @return journey: tuple
        """

        # Remove all empty spaces
        coordinate = coordinate.strip()

        # Remove the parentheses
        coordinate = coordinate.replace('(', '')
        coordinate = coordinate.replace(')', '')

        # split leftover string to a list for each ,
        number_list = coordinate.split(',')

        # The result is a list of 4 numbers
        if len(number_list) != 4:
            raise ValueError

        # Create a tuple from those numbers, for each starting and ending points
        t_1 = tuple([float(number_list[0]), float(number_list[1])])
        t_2 = tuple([float(number_list[2]), float(number_list[3])])

        # The journey is a tuple of starting and ending points
        journey = tuple([t_1, t_2])

        return journey






