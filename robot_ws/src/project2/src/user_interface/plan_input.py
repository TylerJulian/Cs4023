class PlanInput:
    def __init__(self):
        pass

    @staticmethod
    def hello():
        print("hello")

    @staticmethod
    def get_coordinates_from_user():
        print("Please enter a set of coordinates in the following format: ")
        print("((x1,y2),(x2,y2))")
        print("Or enter the START to start")
        while True:
            is_start, u_in = PlanInput.get_input()
            if not is_start:
                x, y = 0, 0
                try:
                    x, y = PlanInput.parse_coordinate(u_in)
                except ValueError:
                    print ("Wrong Format")
                print("X: " + str(x))
                print("Y: " + str(y))


    @staticmethod
    def get_input():
        u_in = raw_input()
        if u_in.lower() == "start":
            return True, u_in
        else:
            return False, u_in


    @staticmethod
    def parse_coordinate(coordinate):
        # we are doing a state machine here
        x_index_start = -1
        x_index_end = -1
        y_index_start = -1
        y_index_end = -1
        state = 0
        index = 0
        while index < len(coordinate):
            if state == 0:
                if coordinate[index] == '(':
                    state = 1
                else:
                    raise ValueError
            elif state == 1:
                if coordinate[index] == '(':
                    x_index_start = index+1
                    state = 2
                else:
                    raise ValueError
            elif state == 2:
                if coordinate[index] == ',':
                    x_index_end = index
                    y_index_start = index+1
                    state = 3
            elif state == 3:
                if coordinate[index] == ')':
                    if coordinate[index+1] == ')':
                        y_index_end = index
                        state = 4
                        break
                    else:
                        raise ValueError
            elif state == 4:
                break
            index += 1

        if x_index_start < 0 or x_index_end < 0 or y_index_start < 0 or y_index_end < 0:
            raise ValueError

        x_str = coordinate[x_index_start:x_index_end]
        y_str = coordinate[y_index_start:y_index_end]

        x = float(x_str)
        y = float(y_str)
        return x, y


