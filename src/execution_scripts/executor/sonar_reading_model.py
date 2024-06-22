

class SonarReadingModel:

    front = True

    right = True

    back = True

    left = True

    def __init__(self, front, right, back, left):
        self.front = front
        self.right = right
        self.back = back
        self.left = left

    def __getitem__(self, key) -> bool:
        match key:
            case 0:
                return self.front
            case 1:
                return self.right
            case 2:
                return self.back
            case 3:
                return self.left
        return False

    def __setitem__(self, key, value) -> None:
        match key:
            case 0:
                self.front = value
            case 1:
                self.right = value
            case 2:
                self.back = value
            case 3:
                self.left = value
