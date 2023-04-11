import os
import csv
from datetime import datetime

def build_action_log_msg(source: str, time_s: int = None, time_ns: int = None, action: int = None, controller_state: int = None):
    return {
        "source": source,
        "time_s": time_s,
        "time_ns": time_ns,
        "action": action,
        "controller_state": controller_state
    }

class Logger:
    def __init__(self, log_dir: str) -> None:
        self.log_dir = log_dir
        self.make_directory()

        # get current time as a string
        timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        self.filename = os.path.join(self.log_dir, f'log_{timestamp}.csv')


    def make_directory(self):
        # check if log_dir is in home
        home = os.path.expanduser('~')
        if not self.log_dir.startswith(home):
            self.log_dir = os.path.join(home, self.log_dir)

        # make directory if it doesn't exist
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)

    def log(self, data: dict):
        # print(self.filename)
        with open(self.filename, 'a') as f:
            writer = csv.writer(f)
            writer.writerow(data.keys())
            writer.writerow(data.values())


if __name__ == '__main__':
    logger = Logger('hri_logs')
    logger.log({'a': 1, 'b': 2})