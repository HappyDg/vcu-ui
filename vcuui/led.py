
class LED_BiColor():
    def __init__(self, path):
        super().__init__()
        self._green_path = path + ':green/brightness'
        self._red_path = path + ':red/brightness'

    def off(self):
        self._set(False, False)

    def red(self):
        self._set(True, False)

    def yellow(self):
        self._set(True, True)

    def green(self):
        self._set(False, True)

    def _set(self, red, green):
        red_str = '1' if red else '0'
        green_str = '1' if green else '0'
        # print(f"led {self._green_path} {red_str} {green_str}")

        with open(self._red_path, 'w+') as f:
            f.write(red_str)

        with open(self._green_path, 'w+') as f:
            f.write(green_str)
