#! /usr/bin/python
import sys
import rospy
import subprocess
#from subprocess import Popen, PIPE, STDOUT

from action_server import Client as ActionClient
from robocup_knowledge import load_knowledge
from grammar_parser import cfgparser

# ----------------------------------------------------------------------------------------------------

#https://stackoverflow.com/questions/287871/print-in-terminal-with-colors
class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

class Parser():
    def __init__(self):
        eegpsr = rospy.get_param('~eegpsr', False)
        if eegpsr:
            self.knowledge = load_knowledge('challenge_eegpsr')
        else:
            self.knowledge = load_knowledge('challenge_gpsr')

        self._load_grammar()

    # ----------------------------------------------------------------------------------------------------

    def _load_grammar(self):
        self.parser = cfgparser.CFGParser.fromstring(self.knowledge.grammar)


    def parse(self, command):
        try:
            return bcolors.OKGREEN \
                   + str(self.parser.parse(self.knowledge.grammar_target, command.strip().split(" "), debug=False)) \
                   + bcolors.ENDC
        except cfgparser.ParseError as e:
            return bcolors.FAIL \
                   + str(e) \
                   + bcolors.ENDC


def main():
    rospy.init_node("test_with_cmdgen")

    # TODO: make generic for multiple robots
    # robot parameter is only set when running roslaunch file
    #robot_name = rospy.get_param('~robot_name')
    #if robot_name == 'amigo':
    #    from robot_skills.amigo import Amigo as Robot
    #elif robot_name == 'sergio':
    #    from robot_skills.sergio import Sergio as Robot
    #else:
    #    raise ValueError('unknown robot')

    from robot_skills.amigo import Amigo as Robot
    robot = Robot()

    action_client = ActionClient(robot.robot_name)
    parser        = Parser()

    command = "amigo go to the bed"

    category = "1"
    # Output returned in error handler
    try:
        print("cmdgen-gpsr output:\n" +
             subprocess.check_output(["/bin/bash", "-i", "-c", "cmdgen-gpsr --bulk 100"]))

    except subprocess.CalledProcessError as e:
        print("cmdgen-gpsr error:\n" + e.output)

    with open('./GPSR Cat1 Examples/GPSR Cat1 Examples.txt') as f:
        lines = [line.strip() for line in f.readlines()]

    command_lines = []
    line_nr = 0
    while line_nr < len(lines):
        line = lines[line_nr]
        if len(line) and line[0] == '#':
            command_lines.append(lines[line_nr + 6].lower())
            line_nr += 7
        else:
            line_nr += 1

    for command in command_lines:
        params = parser.parse(command)
        print (command + ": " + str(params))



    semantics = str(params)  # To have the edits done on params also performed on the semantics.
'''
    try:
        result = action_client.send_task(semantics=semantics)

        if not result.succeeded:
            print "\n    Result from action server:\n\n        {0}\n".format(result)
    except KeyboardInterrupt:
        pass
'''


if __name__ == "__main__":
    sys.exit(main())
